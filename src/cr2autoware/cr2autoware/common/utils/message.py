"""
General Utils
===========================

This module contains general utility functions that are used throughout the package.

---------------------------
"""
# standard imports

# third party
import numpy as np

# ROS message imports
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

# commonroad-io imports
from commonroad.geometry.shape import Circle
from commonroad.geometry.shape import Polygon
from commonroad.geometry.shape import Rectangle

# cr2autoware imports
from .transform import utm2map
from .transform import orientation2quaternion


def create_goal_marker(position) -> Marker:
    """
    Create a ros sphere marker to represent a goal.

    :param position: ros pose.position
    :return: new marker
    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.frame_locked = True
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = position.x
    marker.pose.position.y = position.y
    marker.pose.position.z = position.z

    return marker


def create_goal_region_marker(shape, origin_transformation) -> Marker:
    """
    Create a ros marker to represent a goal_region.

    :param shape: shape(s) of the goal region
    :param origin_transformation: list or array with 2 elements
    :return: new marker
    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "goal_region"
    marker.frame_locked = True
    marker.action = Marker.ADD
    marker.color.r = 1.0
    marker.color.g = 0.843
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.z = 0.0
    if isinstance(shape, Rectangle):
        marker.type = Marker.CUBE
        marker.pose.position = utm2map(origin_transformation, shape.center)
        marker.scale.x = shape.length
        marker.scale.y = shape.width
        marker.scale.z = 0.001
        marker.pose.orientation = orientation2quaternion(shape.orientation)
    elif isinstance(shape, Circle):
        marker.type = Marker.CYLINDER
        marker.pose.position = utm2map(origin_transformation, shape.center)
        marker.scale.x = shape.radius
        marker.scale.y = shape.radius
        marker.scale.z = 0.001
    elif isinstance(shape, Polygon):  # visualizes borders of a goal region
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.15
        points = []
        for v in shape.vertices:
            point = Point()
            point.x = v[0]
            point.y = v[1]
            points.append(point)
        marker.points = points
    return marker


def create_route_marker_msg(path: np.ndarray, velocities: np.ndarray, elevation: float) -> MarkerArray:
    """
    Create a message for a route in rviz Marker.LINE_STRIP format.

    :param path: path for the route
    :param velocities: velocities for the route
    :param elevation: elevation for the route
    :return: route marker message
    """
    route = Marker()
    route.header.frame_id = "map"
    route.id = 1
    route.ns = "route"
    route.frame_locked = True
    route.type = Marker.LINE_STRIP
    route.action = Marker.ADD
    route.scale.x = 0.1
    route.scale.y = 0.1
    route.scale.z = 0.1
    route.color.r = 0.0
    route.color.g = 0.0
    route.color.b = 1.0
    route.color.a = 0.3

    if len(velocities) > 0:
        max_velocity = np.max(velocities)
        if max_velocity < 0.1:
            max_velocity = 0.1

    for i in range(0, len(path)):
        _pt = path[i]
        p = Point()
        p.x = _pt[0]
        p.y = _pt[1]
        p.z = elevation

        if i < len(velocities):
            vel = velocities[i]
        else:
            # change config parameters of velocity smoother if whole path not calculated
            vel = 0

        route.points.append(p)

        c = ColorRGBA()
        c.r = 1.0 * vel / max_velocity
        c.g = 0.0
        c.b = 1.0 - 1.0 * vel / max_velocity
        c.a = 1.0
        route.colors.append(c)

    route_msg = MarkerArray()
    route_msg.markers.append(route)
    return route_msg



def log_obstacle(object_msg, static) -> str:
    """
    Simplify obstacle logging.

    :param object_msg: Object message that contains obstacle information
    :param static: True for static and False for dynamic obstacles
    :return: a string for obstacle logging
    """
    pose = object_msg.initial_state.pose_covariance.pose
    if static:
        return "published a static obstacle at: (%f %f). Dim: (%f, %f)" % (
            pose.position.x,
            pose.position.y,
            object_msg.shape.dimensions.x,
            object_msg.shape.dimensions.y,
        )
    else:
        return (
            "published a dynamic obstacle at: (%f %f); Dim: (%f, %f); velocity: %f; acceleration: %f"
            % (
                pose.position.x,
                pose.position.y,
                object_msg.shape.dimensions.x,
                object_msg.shape.dimensions.y,
                object_msg.initial_state.twist_covariance.twist.linear.x,
                object_msg.initial_state.accel_covariance.accel.linear.x,
            )
        )