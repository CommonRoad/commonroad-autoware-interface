from collections import defaultdict
from dataclasses import dataclass
from logging import Logger

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
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

dict_obstacle_id_to_list_obstacles = defaultdict(list)


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

# commonroad
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.prediction.prediction import TrajectoryPrediction


# typing
from typing import Tuple, Union, List, Dict


class ObstacleOverTime:
    """
    Tracks an obstacle over time. Each instance is one obstacle at one time step.
    The class offers methods to generate commonroad obstacles with correct trajectory.
    """

    # saves instaces based on their id
    _dict_obstacle_id_to_instance = defaultdict()


    @classmethod
    def get_instance_from_obstacle_id(cls, id: int) -> Union["ObstacleOverTime", None]:
        """
        Get instance of class by id or None.
        :param id: id of dynamic obstacle
        :return: instance of obstacle over time correspondint to id
        """
        if(id not in cls._dict_obstacle_id_to_instance.keys()):
            return None
        else:
            return cls._dict_obstacle_id_to_instance[id]

    @classmethod
    def get_converted_dynamic_obstacles(
            cls,
            downsample_steps_ms: float=100
    ) -> List[DynamicObstacle]:
        """
        Get all converted dynamic obstacels.
        :param downsample_steps_ms: downsampling of trajectory points
        :return: list of commonroad dynamic obstacles
        """
        return [oot.generate_obstacle_with_trajectory(
            downsample_ms=downsample_steps_ms
        ) for oot in cls._dict_obstacle_id_to_instance.values()]


    def __init__(self,
                 dynamic_obstacle: DynamicObstacle,
                 time_step: int,
                 initial_ros2_time: Tuple[int, int]
                 ) -> None:
        """
        :param dynamic_obstacle: commonroad dynamic obstacle
        :param time_step: current time step
        :param initial_ros2_time: tuple (sec, nanosec) of initial ros2 time
        """

        # static obstacle params
        self._obstacle_id = dynamic_obstacle.obstacle_id
        self._obstacle_shape = dynamic_obstacle.obstacle_shape
        self._obstacle_type = dynamic_obstacle.obstacle_type
        self._initial_ros2_time: Tuple[int,int] = initial_ros2_time

        # obstacle trajectory over time
        self._dict_time_step_to_state: Dict[int, CustomState] = defaultdict()
        custom_state: CustomState = CustomState(
            position=dynamic_obstacle.prediction.trajectory.state_list[0].position,
            velocity=dynamic_obstacle.prediction.trajectory.state_list[0].velocity,
            orientation=dynamic_obstacle.prediction.trajectory.state_list[0].orientation,
            time_step=dynamic_obstacle.prediction.trajectory.state_list[0].time_step,
            ros2_time_stamp=convert_ros2_time_tuple_to_float(initial_ros2_time)
        )
        self._dict_time_step_to_state[time_step] = custom_state

        # add self instance to class tracking
        self._dict_obstacle_id_to_instance[self._obstacle_id] = self


    @property
    def obstacle_id(self) -> int:
        """
        :return: id of dynamic obstacle
        """
        return self._obstacle_id


    def add_state_at_time_step(self,
                               custom_state: CustomState,
                               time_step: int
                               ) -> None:
        """
        Adds commonroad custom state at time step.
        :param custom_state: CommonRoad custom state
        :param time_step: time step
        """
        self._dict_time_step_to_state[time_step] = custom_state


    def generate_obstacle_with_trajectory(
            self,
            downsample_ms: float=100
    ) -> DynamicObstacle:
        """
        Generates a dynamic obstacle out of the class with correct states
        :param downsample_ms: downsample obstacle trajectories with this delta t
        :return: commonroad dynamic obstacle
        """
        sorted_steps = sorted(list(self._dict_time_step_to_state.keys()), reverse=False)

        # downsample trajectories to meet desired downsample_ms
        state_list: List = list()
        for idx, step in enumerate(sorted_steps):
            current_state = self._dict_time_step_to_state[step]
            current_state.time_step = step
            if(len(state_list) > 1):
                if(current_state.ros2_time_stamp < state_list[-1].ros2_time_stamp + downsample_ms/1000):
                    continue
            state_list.append(current_state)

        # adjust time step of new list to be consecutive integers since downsampling may have deleted some traj. points
        for idx, custom_state in enumerate(state_list):
            custom_state.time_step = state_list[0].time_step + idx
        cr_trajectory = CRTrajectory(state_list[0].time_step, state_list)

        # create trajectory object
        trajectory_prediction = TrajectoryPrediction(
            trajectory=cr_trajectory,
            shape=self._obstacle_shape
        )

        # create initial state
        initial_trajectory_state: CustomState = state_list[0]
        initial_state = InitialState(
            position=initial_trajectory_state.position,
            orientation=initial_trajectory_state.orientation,
            velocity=initial_trajectory_state.velocity,
            acceleration=0.0,
            yaw_rate=0.0,
            slip_angle=0.0,
            time_step=sorted_steps[0],
        )

        # obstacle generation
        dynamic_obstacle = DynamicObstacle(
            obstacle_id=self._obstacle_id,
            obstacle_type=self._obstacle_type,
            obstacle_shape=self._obstacle_shape,
            initial_state=initial_state,
            prediction=trajectory_prediction,
        )

        return dynamic_obstacle





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
                filename=scenario_path
    ).open()

    # Find out how the dynamic obstacles evolve over time
    for time_idx, obstacle_list in enumerate(dynamic_obstacles_per_time_step):
        if(len(obstacle_list) == 0):
            continue

        for obstacle in obstacle_list:
            time_s: float = convert_ros2_time_tuple_to_float(obstacle.ros2_time_stamp)

            custom_state: CustomState = CustomState(
                position=obstacle.prediction.trajectory.state_list[0].position,
                velocity=obstacle.prediction.trajectory.state_list[0].velocity,
                orientation=obstacle.prediction.trajectory.state_list[0].orientation,
                time_step=global_timer.find_closest_time_step(
                    time_s
                ),
                ros2_time_stamp=time_s
            )

            obstacle_over_time = ObstacleOverTime(
                obstacle=obstacle,
                ros2_time=time_s,
                obs_id=obstacle.obstacle_id,
                obs_shape=obstacle.obstacle_shape,
                obs_type=obstacle.obstacle_type,
                custom_state=custom_state,
                time_step=global_timer.find_closest_time_step(
                    time_s
                ),
                temporal_distance_to_step=global_timer.get_distance_to_closest_time_step(time_s)
            )

            dict_obstacle_id_to_list_obstacles[obstacle.obstacle_id].append(obstacle_over_time)

    # Sort list of states for each vehicle in ascending time order
    for _key, _val in dict_obstacle_id_to_list_obstacles.items():
        _val.sort(key=lambda x: x.ros2_time)

    # create dynamic obstacle instances
    dynamic_obstacle_list: List[DynamicObstacle] = [
            create_dynamic_obstacle_from_sorted_states(states=states)
            for obs_id, states in dict_obstacle_id_to_list_obstacles.items()
    ]

    # remove old dynamic obstacles
    for obstacle in scenario.dynamic_obstacles:
        scenario.remove_obstacle(obstacle)

    # add new obstacles
    for idx, dynamic_obstacle in enumerate(dynamic_obstacle_list):
        _logger.info(f'Added {idx + 1}/{len(dynamic_obstacle_list)} obstacle with id: {dynamic_obstacle.obstacle_id}')
        scenario.add_objects(dynamic_obstacle)


    # save file
    file_writer = CommonRoadFileWriter(scenario, planning_problem_set)
    file_writer.write_to_file(save_path, OverwriteExistingFile.ALWAYS)


def create_dynamic_obstacle_from_sorted_states(
        states: List[ObstacleOverTime],
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
    cr_trajectory = CRTrajectory(state_list[0].time_step, [state.custom_state for state in state_list])

    trajectory_prediction = TrajectoryPrediction(
        trajectory=cr_trajectory,
        shape=states[0].obs_shape
    )

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
                obstacle_id=states[0].obs_id,
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
    :param state_list: sorted list of ObstacleOverTime of one vehicle in ascending temporal order
    :return: average shape
    """

    # check if kind of shape changes over time
    for state in states:
        if(type(state.obs_shape) is not type(states[0].obs_shape)):
            _logger.warning(f'the kind of shape for obstacle {states[0].obs_id} changes over course of scenario')
            break

    # average rectangle
    if(isinstance(states[0].obs_shape, Rectangle)):
        avg_width: float = sum([state.obs_shape.width for state in states]) / len(states)
        avg_length: float = sum([state.obs_shape.length for state in states]) / len(states)
        return_shape = Rectangle(
            length=avg_length,
            width=avg_width
        )

    # average circle
    elif(isinstance(states[0].obs_shape, Circle)):
        avg_radius: float = sum([state.obs_shape.radius for state in states]) / len(states)
        return_shape = Circle(
            radius=avg_radius
        )

    # For Polygons do not compute an average, just return the first value
    elif(isinstance(states[0].obs_shape, Polygon)):
        return_shape = states[0].obs_shape

    else:
        raise NotImplementedError(f'shape of type {type(states[0])} not implented.')


    return return_shape








