from collections import defaultdict

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.state import InitialState, CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory

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
        downsample_time_step_ms: float=100
) -> None:
    """
    Adds dynamic obstacles to commonroad scenario from autoware and saves the scenario.
    :param dynamic_obstacles_per_time_step: list of commonroad dynamic obstacles per time step
    :param scenario_path: path to commonroad xml.
    :param save_path: path to save the new scenario to.
    :param downsample_time_step_ms: delta t for downsampling the trajectories of the dynamic obstalces
    """

    scenario, planning_problem_set = CommonRoadFileReader(
                filename=scenario_path
    ).open()

    # Find out how the dynamic obstacles evolve over time
    for time_idx, obstacle_list in enumerate(dynamic_obstacles_per_time_step):
        if(len(obstacle_list) == 0):
            continue

        for obstacle in obstacle_list:
            if(ObstacleOverTime.get_instance_from_obstacle_id(obstacle.obstacle_id) is None):
                # create instance and automatically save in classe if obstacle does not exist
                _ = ObstacleOverTime(
                    obstacle,
                    time_idx,
                    initial_ros2_time=convert_ros2_time_tuple_to_float(obstacle.ros2_time_stamp)
                )
            else:
                # add new trajectory point to existing obstacle
                obstacle_over_time: ObstacleOverTime = ObstacleOverTime.get_instance_from_obstacle_id(obstacle.obstacle_id)
                custom_state: CustomState = CustomState(
                    position=obstacle.prediction.trajectory.state_list[0].position,
                    velocity=obstacle.prediction.trajectory.state_list[0].velocity,
                    orientation=obstacle.prediction.trajectory.state_list[0].orientation,
                    time_step=obstacle.prediction.trajectory.state_list[0].time_step,
                    ros2_time_stamp=convert_ros2_time_tuple_to_float(obstacle.ros2_time_stamp)
                )
                obstacle_over_time.add_state_at_time_step(custom_state, time_idx)


    # Generate obstacle list
    dynamic_obstacle_list: List[DynamicObstacle] = ObstacleOverTime.get_converted_dynamic_obstacles(
        downsample_steps_ms=downsample_time_step_ms
    )

    # remove old dynamic obstacles
    for obstacle in scenario.dynamic_obstacles:
        scenario.remove_obstacle(obstacle)

    # add new obstacles
    for dynamic_obstacle in dynamic_obstacle_list:
        scenario.add_objects(dynamic_obstacle)

    # scenario.add_objects(convert_agent_vehicle(ego_trajectory))

    # save file
    file_writer = CommonRoadFileWriter(scenario, planning_problem_set)
    file_writer.write_to_file(save_path, OverwriteExistingFile.ALWAYS)



def convert_ros2_time_tuple_to_float(
        time_tuple: Union[Tuple[int, int], float]
) -> float:
    """
    Converts ros2 time (= tuple[sec, nanosec) to one float sec value.
    :param time_tuple: time tuple, either directly convertible as [int, float], or from ros2 as [float, float]
    :return: time as float in seconds
    """
    if(isinstance(time_tuple, float)):
        time: float = time_tuple
    else:
        time: float = float(time_tuple[0]) + (float(time_tuple[1]) / 1e9)
    return time





