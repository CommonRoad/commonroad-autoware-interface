# standard imports
import enum
import math
import pathlib
from typing import List

# third party imports
import matplotlib.pyplot as plt
import numpy as np

# ROS imports
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from ament_index_python import get_package_share_directory
import rclpy.logging as ros_logging

# comonroad-io imports
from commonroad.common.util import AngleInterval
from commonroad.geometry.shape import Circle
from commonroad.geometry.shape import Polygon
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import AngleExactOrInterval
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer

# Autoware message imports
from dummy_perception_publisher.msg import Object

# ROS message imports
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

logger = ros_logging.get_logger(__name__)


# TODO next 5 methods -> fix imports, moved to message.py
def create_goal_marker(position):
    """Create a ros sphere marker to represent a goal.

    :param: position: ros pose.position
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


def create_goal_region_marker(shape, origin_transformation):
    """Create a ros marker to represent a goal_region.

    :param: shape: shape(s) of the goal region
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
    """Create a message for a route in rviz Marker.LINE_STRIP format.

    :param path:
    :param velocities:
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


def create_object_base_msg(header, origin_transformation, obstacle):
    """Create a base Object message for static and dynamic obstacles.

    :param header: header message for Object
    :param origin_transformation: list or array with 2 elements
    :param obstacle: CR obstacle
    :return: new Object
    """
    object_msg = Object()
    object_msg.header = header
    pose = Pose()
    pose.position = utm2map(origin_transformation, obstacle.initial_state.position)
    pose.orientation = orientation2quaternion(obstacle.initial_state.orientation)
    object_msg.initial_state.pose_covariance.pose = pose
    object_msg.classification.label = 1
    object_msg.classification.probability = 1.0
    object_msg.shape.dimensions.x = obstacle.obstacle_shape.length
    object_msg.shape.dimensions.y = obstacle.obstacle_shape.width
    object_msg.shape.dimensions.z = 1.5

    return object_msg


def log_obstacle(object_msg, static):
    """Simplify obstacle logging.

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


# _process_dynamic_obs helper method


# TODO move to geometry.py
def traj_linear_interpolate(
    self, point_1: Pose, point_2: Pose, smaller_dt: float, bigger_dt: float
) -> Pose:
    """Interpole a point between two points.

    :param point_1: point which will be smaller than interpolated point (on left-side)
    :param point_1: point which will be bigger than interpolated point (on right-side)
    :param smaller_dt: time step for the point will be interpolated
    :param bigger_dt: time step for the points which will be used for interpolation
    :return: pose of the interpolated point
    """
    new_point = Pose()
    new_point.position.x = point_1.position.x + (
        (point_2.position.x - point_1.position.x) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.position.y = point_1.position.y + (
        (point_2.position.y - point_1.position.y) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.position.z = point_1.position.z + (
        (point_2.position.z - point_1.position.z) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.x = point_1.orientation.x + (
        (point_2.orientation.x - point_1.orientation.x) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.y = point_1.orientation.y + (
        (point_2.orientation.y - point_1.orientation.y) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.z = point_1.orientation.z + (
        (point_2.orientation.z - point_1.orientation.z) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.w = point_1.orientation.w + (
        (point_2.orientation.w - point_1.orientation.w) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    return new_point


# _process_dynamic_obs helper method
# TODO move to geometry.py
def upsample_trajectory(traj, dt_ratio):
    """Compute upsampled trajectory list.

    :param traj: trajectory to compute
    :param dt_ratio: dt_ratio
    """
    point_2 = traj[-1]
    point_1 = traj[-2]
    new_points_x = np.linspace(point_1.position.x, point_2.position.x, dt_ratio)
    new_points_y = np.linspace(point_1.position.y, point_2.position.y, dt_ratio)
    new_points_z = np.linspace(point_1.position.z, point_2.position.z, dt_ratio)
    new_points_ort_x = np.linspace(point_1.orientation.x, point_2.orientation.x, dt_ratio)
    new_points_ort_y = np.linspace(point_1.orientation.y, point_2.orientation.y, dt_ratio)
    new_points_ort_z = np.linspace(point_1.orientation.z, point_2.orientation.z, dt_ratio)
    new_points_ort_w = np.linspace(point_1.orientation.w, point_2.orientation.w, dt_ratio)
    for i in range(
        1, dt_ratio - 1
    ):  # don't take first and last samples, they were already appended
        new_point_pos = Point()
        new_point_pos.x = new_points_x[i]
        new_point_pos.y = new_points_y[i]
        new_point_pos.z = new_points_z[i]
        new_point_ort = Quaternion()
        new_point_ort.x = new_points_ort_x[i]
        new_point_ort.y = new_points_ort_y[i]
        new_point_ort.z = new_points_ort_z[i]
        new_point_ort.w = new_points_ort_w[i]
        new_traj_point = Pose()
        new_traj_point.position = new_point_pos
        new_traj_point.orientation = new_point_ort
        traj.insert(-1, new_traj_point)  # upsampled trajectory list


# TODO fix import -> move to ros_interface.helpers.py
def create_qos_profile(history_policy: QoSHistoryPolicy,
                       reliability_policy: QoSReliabilityPolicy,
                       durability_policy: QoSDurabilityPolicy,
                       depth=1) -> QoSProfile:
    """Creates a ROS Quality-of-Service (QOS) profile with the specified settings"""
    qos_profile = QoSProfile(depth=depth)
    qos_profile.history = history_policy
    qos_profile.reliability = reliability_policy
    qos_profile.durability = durability_policy

    return qos_profile
