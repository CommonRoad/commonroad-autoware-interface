import enum
from typing import List
import math
import numpy as np
import matplotlib.pyplot as plt
from commonroad.geometry.shape import Rectangle
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

def map2utm(self, p: Point) -> np.array:
    """
    Transform position (in autoware) to position (in commonroad).
    :param p: position autoware
    :return: position commonroad
    """
    _x = self.origin_transformation_x + p.x
    _y = self.origin_transformation_y + p.y
    return np.array([_x, _y])

def utm2map(self, position: np.array) -> Point:
    """
    Transform position (in commonroad) to position (in autoware).
    :param position: position commonroad
    :return: position autoware
    """
    p = Point()
    p.x = position[0] - self.origin_transformation_x
    p.y = position[1] - self.origin_transformation_y
    return p