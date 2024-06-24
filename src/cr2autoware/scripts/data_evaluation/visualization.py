import copy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import rgb2hex
from matplotlib import cm

# commonroad
from commonroad.geometry.shape import Circle
from commonroad.geometry.shape import Rectangle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.state import CustomState
from commonroad.scenario.state import InitialState
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory

# typing
from typing import List, Tuple

# cmap for coloring the velocity profile
cmap = cm.get_cmap("plasma")


def visualize_route_and_trajectories(
        scenario_path: str,
        save_path: str,
        reference_trajectory: List[CustomState] = None,
        driven_trajectory: List[CustomState] = None,
        draw_footprint: bool = False,
        draw_ego_trajectory: bool = False,
        draw_reference_trajectory: bool = False,
        save_img: bool = True,
        step: int = None,
        goal_length: float = 6.22125,
        goal_width: float = 4.929569524816457,
        footprint_width: float = 2.253,
        footprint_length: float = 4.977,
        initial_state_radius: float = 0.25,
        initial_state_zorder: float = 50,
        downsample_ms: float = 100,
        saving_format: str = "png"
) -> None:
    """
    Visualizes the reference path with velocity profile, the footprint and the driven trajectory.
    :param scenario_path: path to commonroad scenario
    :param save_path: path to save img to
    :param reference_trajectory: reference trajectory (reference path with velocity profile)
    :param driven_trajectory: driven trajectory
    :param draw_footprint: if true, draws footprint
    :param draw_ego_trajectory: if true, draws ego trejectory
    :param draw_reference_trajectory: if true, draws reference trajectory
    :param save_img: if true, saves image, otherwise displays it in id
    :param step: starting step
    :param goal_length: length of commonroad goal region
    :param goal_width: width of commonroad goal region
    :param footprint_width: width of car footprint
    :param footprint_length: length of car footprint
    :param initial_state_radius: radius of initial state
    :param initial_state_zorder: z-order of initial state
    :param downsample_ms: downsampling of ego trajectory
    :param saving_format: saving format (png or svg)
    """
    plt.cla()
    _ = plt.figure(figsize=(20, 10))

    # open scenario and load planning problem
    scenario, planning_problem_set = CommonRoadFileReader(
        scenario_path
    ).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # get plot limits from reference path
    plot_limits: List[float] = get_plot_limits_from_reference_path(reference_trajectory, margin=20)

    # init renderer for plotting
    draw_params = MPDrawParams()
    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = True
    draw_params.dynamic_obstacle.trajectory.zorder = initial_state_zorder
    if(step is not None):
        draw_params.time_begin = step
    renderer = MPRenderer(plot_limits=plot_limits)
    renderer.draw_params.dynamic_obstacle.draw_icon = True
    renderer.draw_params.dynamic_obstacle.trajectory.draw_trajectory = True
    renderer.draw_params.planning_problem.initial_state.state.draw_arrow = False
    renderer.draw_params.planning_problem.initial_state.state.radius = initial_state_radius
    renderer.draw_params.planning_problem.initial_state.state.zorder = initial_state_zorder
    planning_problem.goal.state_list[0].position.length = goal_length
    planning_problem.goal.state_list[0].position.width = goal_width
    planning_problem.draw(renderer)

    # downsample driven trajectory to scenario time step for visualization
    driven_traj_downsampled = list()
    if driven_trajectory is not None:
        ros2_time_stamp_seconds_list = [state.ros2_time_stamp[0] + state.ros2_time_stamp[1]/1e9 for state in driven_trajectory]
        driven_traj_downsampled.append(driven_trajectory[0])
        curr_time_stamp = ros2_time_stamp_seconds_list[0]
        for i in range(1, len(ros2_time_stamp_seconds_list)):
            if ros2_time_stamp_seconds_list[i] < curr_time_stamp + (downsample_ms / 1000):
                continue
            else:
                curr_time_stamp = ros2_time_stamp_seconds_list[i]
                driven_traj_downsampled.append(driven_trajectory[i])

    # draw ego vehicle footprint
    if driven_trajectory is not None and draw_footprint:
        if draw_footprint:
            # icon for first state
            draw_ego_vehicle(
                renderer,
                driven_traj_downsampled,
                width=footprint_width,
                length=footprint_length
            )

    # draw velocity profile and reference path
    if reference_trajectory is not None and draw_reference_trajectory:
        v_min, v_max = get_velocity_min_max_from_trajectory(reference_trajectory)
        for state in reference_trajectory:
            draw_route_state(
                renderer,
                state,
                v_min,
                v_max
            )

    # draw driven trajectory
    if driven_trajectory is not None and draw_ego_trajectory:
        for state in driven_traj_downsampled:
            draw_car_state(
                renderer,
                state
            )

    # draw scenario and renderer
    scenario.draw(renderer, draw_params=draw_params)
    renderer.render()
    plt.axis('off')

    # save or show scenario
    if save_img:
        plt.savefig(save_path, format=saving_format, dpi=300, bbox_inches='tight')
    else:
        plt.show()


def draw_route_state(
        renderer: MPRenderer,
        state: CustomState,
        v_min: float,
        v_max: float,
        point_radius: float=0.1
) -> None:
    """
    Draws global trajectory and color-codes velocity profile.
    :param renderer: commonroad renderer object
    :param state: commonroad custom state to render
    :param v_min: minimum velocity in velocity profile
    :param v_max: maximum velocity in velocity profile
    :param point_radius: radius of displayed points in meters
    """
    normalized_velocity: float = (state.velocity - v_min) / (v_max - v_min)
    rbg_color = cmap(normalized_velocity)
    hex_color = rgb2hex(rbg_color)
    draw_params = copy.copy(renderer.draw_params)
    draw_params.shape.facecolor = hex_color
    draw_params.shape.edgecolor = hex_color

    occ_pos = Circle(radius=point_radius, center=state.position)
    occ_pos.draw(renderer, draw_params=draw_params)


def draw_ego_vehicle(
        renderer: MPRenderer,
        state_list: List[CustomState],
        length: float,
        width: float,
        opacity: float = 0.05,
        face_color: str = "#E37222",
        edge_color: str = "#9C4100"
) -> None:
    """
    Draws the ego vehicle at its initial state with a car icon and the footprints of its trajectory
    :param renderer: CommonRoad renderer
    :param state_list: List of CommonRoad custom state
    :param length: length of car
    :param width: width of car
    :param opacity: opacity of displayed footprint
    :param face_color: face color of the car
    :param edge_color: edge color of the car
    """
    # draw footprints for occupancy
    vel_array = np.array([state.velocity for state in state_list])
    idx_start = np.argmax(vel_array > 0)
    for state in state_list[idx_start:]:
        draw_car_footprint(
            renderer,
            state,
            width=width,
            length=length,
            opacity=opacity,
            face_color=face_color
        )

    # create ego dynamic obstacle
    ego_type = ObstacleType.CAR
    ego_shape = Rectangle(length=length,
                          width=width)
    ego_init_state = InitialState(time_step=state_list[0].time_step,
                                  position=state_list[0].position,
                                  orientation=state_list[0].orientation,
                                  velocity=state_list[0].velocity,
                                  acceleration=0.0,
                                  yaw_rate=0.0,
                                  slip_angle=0.0)
    ego_trajectory = Trajectory(5, state_list[1:])
    ego_prediction = TrajectoryPrediction(trajectory=ego_trajectory, shape=ego_shape)

    ego = DynamicObstacle(obstacle_id=9999,
                          obstacle_type=ego_type,
                          obstacle_shape=ego_shape,
                          initial_state=ego_init_state,
                          prediction=ego_prediction)

    # draw icon for initial state
    draw_params = copy.copy(renderer.draw_params)
    draw_params.dynamic_obstacle.vehicle_shape.occupancy.facecolor = face_color
    draw_params.dynamic_obstacle.vehicle_shape.occupancy.edgecolor = edge_color
    draw_params.dynamic_obstacle.draw_initial_state = False
    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
    ego.draw(renderer, draw_params=draw_params)


def draw_car_footprint(
        renderer: MPRenderer,
        state: CustomState,
        length: float,
        width: float,
        opacity: float = 0.1,
        face_color: str = "#E37222",
        edge_color: str = "#d64c13"
) -> None:
    """
    Draws the car state as footprint.
    :param renderer: commonroad renderer
    :param state: commonroad custom state
    :param length: length of car
    :param width: width of car
    :param opacity: opacity of displayed footprint
    :param face_color: face color of displayed footprint
    :param edge_color: edge color of displayed footprint
    """
    # Draw footprint
    draw_params = copy.copy(renderer.draw_params)
    draw_params.shape.facecolor = face_color
    draw_params.shape.edgecolor = edge_color
    draw_params.shape.opacity = opacity
    rectangle = Rectangle(length=length,
                          width=width,
                          center=state.position,
                          orientation=state.orientation)
    rectangle.draw(renderer, draw_params=draw_params)


def draw_car_state(
        renderer: MPRenderer,
        state: CustomState,
        point_radius: float = 0.1,
        face_color: str = "#000000",
        edge_color: str = "#000000",
        opacity: float = 1.0
) -> None:
    """
    :param renderer: commonroad renderer
    :param state: commonroad custom state
    :param point_radius: radius of displayed point
    :param face_color: face color of displayed point
    :param edge_color: edge color of displayed point
    :param opacity: opacity of displayed point
    """
    draw_params = copy.copy(renderer.draw_params)
    draw_params.shape.facecolor = face_color
    draw_params.shape.edgecolor = edge_color
    draw_params.shape.opacity = opacity
    occ_pos = Circle(radius=point_radius, center=state.position)
    occ_pos.draw(renderer, draw_params=draw_params)


def get_velocity_min_max_from_trajectory(
        trajectory:List[CustomState]
) -> Tuple[float, float]:
    """
    Gets min and max velocity from global trajectory for color coding.
    :param trajectory: list of commonroad custom states in ascending temporal order with velocity attribute.
    :return: tuple[v_min, v_max]
    """
    min_velocity: float = min(trajectory, key=lambda x: x.velocity).velocity
    max_velocity: float = max(trajectory, key=lambda x: x.velocity).velocity
    return (min_velocity, max_velocity)


def get_plot_limits_from_reference_path(
        reference_path: List[CustomState],
        margin: float = 5
) -> List[float]:
    """
    Computes plot limits from reference-path
    :param reference_path: list of commonroad custom states
    :param margin: margin to display
    :return: [x_in, x_max, y_min, y_max]
    """
    b_x_min: CustomState = min(reference_path, key=lambda x: x.position[0])
    b_x_max: CustomState = max(reference_path, key=lambda x: x.position[0])
    b_y_min: CustomState = min(reference_path, key=lambda x: x.position[1])
    b_y_max: CustomState = max(reference_path, key=lambda x: x.position[1])

    x_min = np.min(b_x_min.position[0]) - margin
    x_max = np.max(b_x_max.position[0]) + margin
    y_min = np.min(b_y_min.position[1]) - margin
    y_max = np.max(b_y_max.position[1]) + margin
    return [x_min, x_max, y_min, y_max]
















