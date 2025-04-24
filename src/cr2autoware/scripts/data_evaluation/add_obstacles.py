from collections import defaultdict
from dataclasses import dataclass
from logging import Logger

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.util import FileFormat
from commonroad.scenario.state import InitialState, CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.geometry.shape import (
    Circle,
    Polygon,
    Rectangle,
    Shape
)
from commonroad.scenario.obstacle import ObstacleType

# own code base
from global_timer import GlobalTimer
from data_eval_utils import convert_ros2_time_tuple_to_float

# typing
from typing import List, Any, Union, Dict


_logger = Logger(__name__)


@dataclass
class ObstacleOverTime:
    obstacle: DynamicObstacle
    ros2_time: float
    time_step: int
    obs_id: int
    obs_shape: Any
    obs_type: Any
    custom_state: CustomState
    temporal_distance_to_step: float


def add_dynamic_obstacles(
        dynamic_obstacles_per_time_step: List[List[DynamicObstacle]],
        scenario_path: str,
        save_path: str,
        global_timer: GlobalTimer
) -> None:
    """
    Adds dynamic obstacles to commonroad scenario from autoware and saves the scenario.
    :param dynamic_obstacles_per_time_step: list of commonroad dynamic obstacles per time step
    :param scenario_path: path to commonroad xml.
    :param save_path: path to save the new scenario to.
    :param global_timer: global time for scenario
    """

    scenario, planning_problem_set = CommonRoadFileReader(
                filename_2020a=scenario_path
    ).open()

    # Find out how the dynamic obstacles evolve over time
    dict_obstacle_id_to_list_obstacles = defaultdict(list)
    for time_idx, obstacle_list in enumerate(dynamic_obstacles_per_time_step):
        if(len(obstacle_list) == 0):
            continue

        for obstacle in obstacle_list:
            time_s: float = convert_ros2_time_tuple_to_float(obstacle.ros2_time_stamp)
            time_step = global_timer.find_closest_time_step(time_s)

            custom_state: CustomState = CustomState(
                position=obstacle.initial_state.position,
                velocity=obstacle.initial_state.velocity,
                orientation=obstacle.initial_state.orientation,
                time_step=time_step,
                ros2_time_stamp=time_s
            )

            obstacle_over_time = ObstacleOverTime(
                obstacle=obstacle,
                ros2_time=time_s,
                obs_id=obstacle.obstacle_id,
                obs_shape=obstacle.obstacle_shape,
                obs_type=obstacle.obstacle_type,
                custom_state=custom_state,
                time_step=time_step,
                temporal_distance_to_step=global_timer.get_distance_to_closest_time_step(time_s)
            )

            dict_obstacle_id_to_list_obstacles[obstacle.obstacle_id].append(obstacle_over_time)

    # Sort list of states for each vehicle in ascending time order
    for _key, _val in dict_obstacle_id_to_list_obstacles.items():
        _val.sort(key=lambda x: x.ros2_time)

    # create dynamic obstacle instances and re-map IDs
    dynamic_obstacle_list: List[DynamicObstacle] = [
            create_dynamic_obstacle_from_sorted_states(states=states, obs_id=obs_id + 1000)
            for obs_id, (_, states) in enumerate(sorted(dict_obstacle_id_to_list_obstacles.items(), key=lambda x: x[0]))
    ]

    # remove old dynamic obstacles
    for obstacle in scenario.dynamic_obstacles:
        scenario.remove_obstacle(obstacle)

    # add new obstacles
    for idx, dynamic_obstacle in enumerate(dynamic_obstacle_list):
        _logger.info(f'Added {idx + 1}/{len(dynamic_obstacle_list)} obstacle with id: {dynamic_obstacle.obstacle_id}')
        scenario.add_objects(dynamic_obstacle)


    # save file
    file_writer = CommonRoadFileWriter(scenario, planning_problem_set, file_format=FileFormat.XML)
    file_writer.write_to_file(save_path, OverwriteExistingFile.ALWAYS)


def create_dynamic_obstacle_from_sorted_states(
        states: List[ObstacleOverTime], obs_id: int
) -> DynamicObstacle:
    """
    Creates dynamic obstacle from sorted list of states.
    :param states: list of ObstacleOverTime instances for one vehicle in ascending time order
    :param global_timer: global timer instance
    :return: dynamic obstacle
    """

    # Filter out double entries in steps by taking the ones closest to the time steps continuous time value
    dict_step_to_obsovertime = defaultdict(list)
    state_list: List[ObstacleOverTime] = list()
    for state in states:
        dict_step_to_obsovertime[state.time_step].append(state)
    for step, states in dict_step_to_obsovertime.items():
        state_list.append(min(states, key=lambda x: x.temporal_distance_to_step))

    # shape
    shape = calculate_average_shape(states=state_list)

    # obstacle type
    obstacle_type = calculate_obstacle_type(states=state_list)

    # create trajectory object
    if len(state_list) > 1:
        cr_trajectory = CRTrajectory(state_list[1].time_step, [state.custom_state for state in state_list[1:]])
        trajectory_prediction = TrajectoryPrediction(
            trajectory=cr_trajectory,
            shape=shape,
        )
    else:
        trajectory_prediction = None

    # create initial state
    initial_trajectory_state: CustomState = state_list[0].custom_state
    initial_state = InitialState(
        position=initial_trajectory_state.position,
        orientation=initial_trajectory_state.orientation,
        velocity=initial_trajectory_state.velocity,
        acceleration=0.0,
        yaw_rate=0.0,
        slip_angle=0.0,
        time_step=initial_trajectory_state.time_step,
    )

    # obstacle generation
    return DynamicObstacle(
                obstacle_id=obs_id,
                obstacle_type=obstacle_type,
                obstacle_shape=shape,
                initial_state=initial_state,
                prediction=trajectory_prediction,
            )


def calculate_obstacle_type(
    states: List[ObstacleOverTime]
) -> ObstacleType:
    """
    Calculates obstacle type
    :param states: sorted list of ObstacleOverTime of one vehicle in ascending order
    :return: average shape
    """
    # check if obstacle type changes
    for state in states:
        if(state.obs_type is not states[0].obs_type):
            _logger.warning(f'the typ for obstacle {states[0].obs_id} changes over course of scenario')
            break

    # Pseudo Histogram
    dict_obs_type_to_counter: Dict[ObstacleType, int] = {
        ObstacleType.UNKNOWN: 0,
        ObstacleType.CAR: 0,
        ObstacleType.TRUCK: 0,
        ObstacleType.BUS: 0,
        ObstacleType.MOTORCYCLE: 0,
        ObstacleType.BICYCLE: 0,
        ObstacleType.PEDESTRIAN: 0,
    }
    for state in states:
        dict_obs_type_to_counter[state.obs_type] += 1

    # return key with highest value
    return max(dict_obs_type_to_counter, key=dict_obs_type_to_counter.get)



def calculate_average_shape(
        states: List[ObstacleOverTime],
) -> Union[Shape, Rectangle, Circle, Polygon]:
    """
    Calculates average shape, except for polygons (uses first shape).
    :param states: sorted list of ObstacleOverTime of one vehicle in ascending temporal order
    :return: average shape
    """

    # check if kind of shape changes over time
    for state in states:
        if(type(state.obs_shape) is not type(states[0].obs_shape)):
            _logger.warning(f'the kind of shape for obstacle {states[0].obs_id} changes over course of scenario')
            break

    shape_cnt = {
        "rectangle": [],
        "circle": [],
        "polygon": [],
        "other": [],
    }
    for state in states:
        if isinstance(state.obs_shape, Rectangle):
            shape_cnt["rectangle"].append(state.obs_shape)
        elif isinstance(state.obs_shape, Circle):
            shape_cnt["circle"].append(state.obs_shape)
        elif isinstance(state.obs_shape, Polygon):
            shape_cnt["polygon"].append(state.obs_shape)
        else:
            shape_cnt["other"].append(state.obs_shape)
    shapes = max(shape_cnt.items(), key=lambda x: len(x[1]))[1]

    # average rectangle
    if isinstance(shapes[0], Rectangle):
        avg_width: float = sum(shape.width for shape in shapes) / len(shapes)
        avg_length: float = sum(shape.length for shape in shapes) / len(shapes)
        return_shape = Rectangle(
            length=avg_length,
            width=avg_width
        )

    # average circle
    elif isinstance(shapes[0], Circle):
        avg_radius: float = sum(shape.radius for shape in shapes) / len(shapes)
        return_shape = Circle(
            radius=avg_radius
        )

    # For Polygons do not compute an average, just return the first value
    elif isinstance(shapes[0], Polygon):
        return_shape = shapes[0]

    else:
        raise NotImplementedError(f'shape of type {type(states[0])} not implented.')


    return return_shape








