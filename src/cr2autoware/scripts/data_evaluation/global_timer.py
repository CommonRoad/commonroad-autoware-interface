import numpy as np

# commonroad
from commonroad.scenario.traffic_light import TrafficLight
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.planning.planning_problem import Trajectory

# own code base
from data_eval_utils import convert_ros2_time_tuple_to_float

# typing
from typing import List, Tuple, Union



class GlobalTimer:
    """
    Global timer for the created scenario
    """

    def __init__(
            self,
            driven_trajectory: Union[None, Trajectory],
            traffic_light_data: Union[None, List[Tuple[Tuple[int, int], List[TrafficLight]]]],
            predicted_obstacles: Union[None, List[DynamicObstacle]],
            downsample_ms: int
    ) -> None:
        """
        Takes different pickle (.pkl) files as input and computes timer value
        :param driven_trajectory: data from pkl of driven trajectory
        :param traffic_light_data: data from pkl for traffic lights
        :param predicted_obstacles: data from pkl for predicted obstacles
        :param downsample_ms: downsampling in miliseconds
        """

        self.time_start: float
        self.time_end: float

        self._calc_start_end_time(
            driven_trajectory=driven_trajectory,
            traffic_light_data=traffic_light_data,
            predicted_obstacles=predicted_obstacles
        )

        self.downsample_ms: int = downsample_ms
        self.time_steps: List[float] = np.arange(self.time_start, self.time_end, self.downsample_ms/1e3).tolist()
        self.final_state_idx: int = len(self.time_steps) - 1
        self.initial_state_idx: int = 0




    def find_closest_time_step(self, time_s: float) -> int:
        """
        Takes time in seconds as float and finds closest time step

        :param time_s: continous time value in seconds.
        """
        return self.time_steps.index(min(self.time_steps, key=lambda x: abs(x - time_s)))


    def get_distance_to_closest_time_step(self, time_s: float) -> float:
        """
        Calculates distance to nearest time step
        :param time_s: continous time in seconds
        :return: distance to closest index
        """
        return min(self.time_steps, key=lambda x: abs(x - time_s))


    # TODO make more modulare so that traffic lights or traffic participants can also be empty lists
    def _calc_start_end_time(
            self,
            driven_trajectory: Trajectory,
            traffic_light_data: List[Tuple[Tuple[int, int], List[TrafficLight]]],
            predicted_obstacles: List[DynamicObstacle],
    ) -> None:
        """
        Takes different pickle (.pkl) files as input and computes timer value
        :param driven_trajectory: data from pkl of driven trajectory
        :param traffic_light_data: data from pkl for traffic lights
        :param predicted_obstacles: data from pkl for predicted obstacles
        :param downsample_ms: downsampling in miliseconds
        """
        min_vals: List[float] = list()
        max_vals: List[float] = list()

        # driven trajectory
        if(driven_trajectory is not None):
            t_min_driven_traj: float = convert_ros2_time_tuple_to_float(
                min(
                driven_trajectory.state_list,
                key=lambda x: convert_ros2_time_tuple_to_float(x.ros2_time_stamp)
                ).ros2_time_stamp
            )
            t_max_driven_traj: float = convert_ros2_time_tuple_to_float(
                max(
                driven_trajectory.state_list,
                key=lambda x: convert_ros2_time_tuple_to_float(x.ros2_time_stamp)
                ).ros2_time_stamp
            )

            min_vals.append(t_min_driven_traj)
            max_vals.append(t_max_driven_traj)


        # obstacles
        if(predicted_obstacles is not None):
            all_obstacles: List[DynamicObstacle] = list()
            for el in predicted_obstacles:
                all_obstacles.extend(el)
            t_min_obs: float = convert_ros2_time_tuple_to_float(
                min(
                all_obstacles,
                key=lambda x: convert_ros2_time_tuple_to_float(x.ros2_time_stamp)
                ).ros2_time_stamp
            )
            t_max_obs: float = convert_ros2_time_tuple_to_float(
                max(
                    all_obstacles,
                    key=lambda x: convert_ros2_time_tuple_to_float(x.ros2_time_stamp)
                ).ros2_time_stamp
            )
            min_vals.append(t_min_obs)
            max_vals.append(t_max_obs)

        if(traffic_light_data is not None):
            # traffic lights
            traffic_lights_time: List[float] = [
                convert_ros2_time_tuple_to_float(x[0])
                for x in traffic_light_data
            ]
            t_min_traffic_lights: float = min(traffic_lights_time)
            t_max_traffic_lights: float = max(traffic_lights_time)

            min_vals.append(t_min_traffic_lights)
            max_vals.append(t_max_traffic_lights)



        # calc time
        self.time_start = min(min_vals)
        self.time_end = max(max_vals)