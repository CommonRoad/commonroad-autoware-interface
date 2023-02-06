import enum
from typing import List
import math
import numpy as np
import matplotlib.pyplot as plt
from commonroad.geometry.shape import Rectangle, Circle, Polygon
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State, Trajectory
# import CommonRoad-io modules
from commonroad.visualization.mp_renderer import MPRenderer

# ROS message imports
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header

# Autoware message imports
from dummy_perception_publisher.msg import Object


@enum.unique
class MotionPrimitiveStatus(enum.Enum):
    IN_FRONTIER = 0
    INVALID = 1
    CURRENTLY_EXPLORED = 2
    EXPLORED = 3
    SOLUTION = 4


def plot_primitive_path(mp: List[State], status: MotionPrimitiveStatus, plotting_params):
    plt.plot(mp[-1].position[0], mp[-1].position[1], color=plotting_params[status.value][0], marker='o', markersize=8,
             zorder=27)
    x = []
    y = []
    for state in mp:
        x.append(state.position[0])
        y.append(state.position[1])
    plt.plot(x, y, color=plotting_params[status.value][0], marker="", linestyle=plotting_params[status.value][1],
             linewidth=plotting_params[status.value][2], zorder=25)


def display_steps(scenario_data, algorithm, config, **args):
    scenario = scenario_data[0]
    initial_state = scenario_data[1]
    ego_shape = scenario_data[2]
    planning_problem = scenario_data[3]

    plt.figure()
    plt.axis('equal')
    renderer = MPRenderer()
    draw_params = {'scenario': {'lanelet': {'facecolor': '#F8F8F8'}}}
    scenario.draw(renderer, draw_params=draw_params)

    ego_vehicle = DynamicObstacle(obstacle_id=scenario.generate_object_id(), obstacle_type=ObstacleType.CAR,
                                  obstacle_shape=ego_shape,
                                  initial_state=initial_state)
    ego_vehicle.draw(renderer)
    planning_problem.draw(renderer)

    renderer.render()

    if 'limit_depth' in args:
        path, primitives, list_states_nodes = algorithm(limit_depth=args['limit_depth'])
    else:
        path, primitives, list_states_nodes = algorithm()

    for node_status in list_states_nodes:
        for node in node_status.values():
            plot_primitive_path(node[0], node[1], config.PLOTTING_PARAMS)
            plt.pause(0.1)


def visualize_solution(scenario: Scenario,
                       planning_problem_set: PlanningProblemSet,
                       trajectory: Trajectory) -> None:
    num_time_steps = len(trajectory.state_list)

    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_initial_state = trajectory.state_list[0]
    dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
    dynamic_obstacle_prediction = TrajectoryPrediction(trajectory, dynamic_obstacle_shape)

    # generate the dynamic obstacle according to the specification
    dynamic_obstacle_id = scenario.generate_object_id()
    dynamic_obstacle_type = ObstacleType.CAR
    dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                       dynamic_obstacle_type,
                                       dynamic_obstacle_shape,
                                       dynamic_obstacle_initial_state,
                                       dynamic_obstacle_prediction)

    # visualize scenario
    plt.figure()
    renderer = MPRenderer()
    for i in range(0, num_time_steps):
        renderer.clear()
        scenario.draw(renderer, draw_params={'time_begin': i})
        planning_problem_set.draw(renderer)
        dynamic_obstacle.draw(renderer, draw_params={'time_begin': i,
                                                     'dynamic_obstacle': {'shape': {'facecolor': 'green'},
                                                                          'trajectory': {'draw_trajectory': True,
                                                                                         'facecolor': '#ff00ff',
                                                                                         'draw_continuous': True,
                                                                                         'z_order': 60,
                                                                                         'line_width': 5}
                                                                          }
                                                     })

        plt.gca().set_aspect('equal')
        renderer.render()
        plt.pause(0.1)

def orientation2quaternion(orientation: float) -> Quaternion:
    """
    Transform orientation (in commonroad) to quaternion (in autoware).
    :param orientation: orientation angles
    :return: orientation quaternion
    """
    quat = Quaternion()
    quat.w = math.cos(orientation * 0.5)
    quat.z = math.sin(orientation * 0.5)
    return quat

def quaternion2orientation(quaternion: Quaternion) -> float:
    """
    Transform quaternion (in autoware) to orientation (in commonroad).
    :param quaternion: orientation quaternion
    :return: orientation angles
    """
    z = quaternion.z
    w = quaternion.w
    mag2 = (z * z) + (w * w)
    epsilon = 1e-6
    if abs(mag2 - 1.0) > epsilon:
        mag = 1.0 / math.sqrt(mag2)
        z *= mag
        w *= mag

    y = 2.0 * w * z
    x = 1.0 - 2.0 * z * z
    return math.atan2(y, x)

def map2utm(origin_transformation, p: Point) -> np.array:
    """
    Transform position (in autoware) to position (in commonroad).
    :param origin_transformation: list or array with 2 elements
    :param p: position autoware
    :return: position commonroad
    """
    _x = origin_transformation[0] + p.x
    _y = origin_transformation[1] + p.y
    return np.array([_x, _y])

def utm2map(origin_transformation, position: np.array) -> Point:
    """
    Transform position (in commonroad) to position (in autoware).
    :param origin_transformation: list or array with 2 elements
    :param position: position commonroad
    :return: position autoware
    """
    p = Point()
    p.x = position[0] - origin_transformation[0]
    p.y = position[1] - origin_transformation[1]
    return p

def create_goal_marker(position):
    """
    creates a ros sphere marker to represent a goal
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
    """
    creates a ros marker to represent a goal_region
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
    elif isinstance(shape, Polygon): # visualizes borders of a goal region
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

def create_route_marker_msg(path, velocities):
    """
    creates a message for a route in rviz Marker.LINE_STRIP format
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

    if path != []:
        max_velocity = max(velocities)
        if max_velocity < 0.1:
            max_velocity = 0.1

    for i in range(0, len(path)):
        point = path[i]
        if i < len(velocities):
            vel = velocities[i]
        else:
            # change config parameters of velocity smoother if whole path not calculated
            vel = 0

        #p = utm2map(self.origin_transformation, point)
        #p.z = 0
        p = point
        route.points.append(p)

        c = ColorRGBA()
        c.r = 1.0 * vel / max_velocity
        c.g = 0.0
        c.b = 1.0 - 1.0 * vel/ max_velocity
        c.a = 1.0
        route.colors.append(c)

    route_msg = MarkerArray()
    route_msg.markers.append(route)
    return route_msg

def create_object_base_msg(header, origin_transformation, obstacle):
    """
    creates a base Object message for static and dynamic obstacles
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
    """
    simplifies obstacle logging
    :param object_msg: Object message that contains obstacle information
    :param static: True for static and False for dynamic obstacles
    :return: a string for obstacle logging
    """
    pose = object_msg.initial_state.pose_covariance.pose
    if static:
        return "published a static obstacle at: (%f %f). Dim: (%f, %f)" % (
                pose.position.x, pose.position.y,
                object_msg.shape.dimensions.x, object_msg.shape.dimensions.y
                )
    else:
        return "published a dynamic obstacle at: (%f %f); Dim: (%f, %f); velocity: %f; acceleration: %f" % (
                pose.position.x, pose.position.y,
                object_msg.shape.dimensions.x, object_msg.shape.dimensions.y,
                object_msg.initial_state.twist_covariance.twist.linear.x,
                object_msg.initial_state.accel_covariance.accel.linear.x
                )

# _process_dynamic_obs helper method

def traj_linear_interpolate(self, point_1: Pose, point_2: Pose, smaller_dt: float, bigger_dt: float) -> Pose:
    """
    interpolation for a point between two points
    :param point_1: point which will be smaller than interpolated point (on left-side)
    :param point_1: point which will be bigger than interpolated point (on right-side)
    :param smaller_dt: time step for the point will be interpolated
    :param bigger_dt: time step for the points which will be used for interpolation
    :return: pose of the interpolated point
    """
    new_point = Pose()
    new_point.position.x = point_1.position.x + \
                            ((point_2.position.x - point_1.position.x) / smaller_dt) * \
                            (bigger_dt - smaller_dt)
    new_point.position.y = point_1.position.y + \
                            ((point_2.position.y - point_1.position.y) / smaller_dt) * \
                            (bigger_dt - smaller_dt)
    new_point.position.z = point_1.position.z + \
                            ((point_2.position.z - point_1.position.z) / smaller_dt) * \
                            (bigger_dt - smaller_dt)
    new_point.orientation.x = point_1.orientation.x + \
                                ((point_2.orientation.x - point_1.orientation.x) / smaller_dt) * \
                                (bigger_dt - smaller_dt)
    new_point.orientation.y = point_1.orientation.y + \
                                ((point_2.orientation.y - point_1.orientation.y) / smaller_dt) * \
                                (bigger_dt - smaller_dt)
    new_point.orientation.z = point_1.orientation.z + \
                                ((point_2.orientation.z - point_1.orientation.z) / smaller_dt) * \
                                (bigger_dt - smaller_dt)
    new_point.orientation.w = point_1.orientation.w + \
                                ((point_2.orientation.w - point_1.orientation.w) / smaller_dt) * \
                                (bigger_dt - smaller_dt)
    return new_point

# _process_dynamic_obs helper method
def upsample_trajectory(traj, dt_ratio):
    """
    computes upsampled trajectory list
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
    for i in range(1, dt_ratio - 1):  # don't take first and last samples, they were already appended
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
