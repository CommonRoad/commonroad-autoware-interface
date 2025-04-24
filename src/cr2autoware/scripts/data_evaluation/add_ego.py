import itertools
from collections import defaultdict
from typing import List

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.util import FileFormat
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory

from scripts.data_evaluation.data_eval_utils import convert_ros2_time_tuple_to_float
from scripts.data_evaluation.global_timer import GlobalTimer


def add_ego_vehicle(
    driven_trajectory: List[CustomState],
    scenario_path: str,
    save_path: str,
    global_timer: GlobalTimer,
) -> None:

    scenario, planning_problem_set = CommonRoadFileReader(
        filename_2020a=scenario_path
    ).open()

    cr_states = defaultdict(list)
    for state in driven_trajectory:
        time_s = convert_ros2_time_tuple_to_float(state.ros2_time_stamp)
        time_step = global_timer.find_closest_time_step(time_s)
        cr_state = CustomState(
            time_step=time_step,
            position=state.position,
            orientation=state.orientation,
            velocity=state.velocity,
            yaw_rate=state.yaw_rate,
        )
        time_distance = global_timer.get_distance_to_closest_time_step(time_s)
        cr_states[time_step].append((cr_state, time_distance))

    cr_state_list = [
        min(states, key=lambda x: x[1])[0]
        for _, states in sorted(cr_states.items(), key=lambda x: x[0])
    ]

    # create ego dynamic obstacle
    ego_type = ObstacleType.CAR
    width = 2.253
    length = 4.977
    ego_shape = Rectangle(length=length, width=width)
    ego_init_state = InitialState(
        time_step=cr_state_list[0].time_step,
        position=cr_state_list[0].position,
        orientation=cr_state_list[0].orientation,
        velocity=cr_state_list[0].velocity,
        acceleration=0.0,
        yaw_rate=cr_state_list[0].yaw_rate,
        slip_angle=0.0,
    )

    ego_trajectory = Trajectory(cr_state_list[1].time_step, cr_state_list[1:])
    ego_prediction = TrajectoryPrediction(trajectory=ego_trajectory, shape=ego_shape)

    ego = DynamicObstacle(
        obstacle_id=42,
        obstacle_type=ego_type,
        obstacle_shape=ego_shape,
        initial_state=ego_init_state,
        prediction=ego_prediction,
    )
    scenario.add_objects(ego)

    driving_time_steps = [
        list(time_step for time_step, _ in time_steps_with_driving)
        for is_driving, time_steps_with_driving in itertools.groupby(
            ((state.time_step, state.velocity > 0) for state in cr_state_list),
            key=lambda x: x[1],
        )
        if is_driving
    ]

    start_time_step = next(
        (time_steps[0] for time_steps in driving_time_steps if len(time_steps) > 5),
        None,
    )
    if start_time_step is None:
        raise RuntimeError("Ego vehicle is never driving for 5 consecutive time steps.")
    start_time_step = max(start_time_step - 1, 0)

    to_remove = []
    for dyn_obs in scenario.dynamic_obstacles:
        state_list = [dyn_obs.initial_state]
        if dyn_obs.prediction is not None:
            state_list += dyn_obs.prediction.trajectory.state_list
        if state_list[0].time_step < start_time_step:
            idx_first_state = next(
                (
                    i
                    for i, state in enumerate(state_list)
                    if state.time_step >= start_time_step
                ),
                None,
            )

            if idx_first_state is None:
                to_remove.append(dyn_obs)
            else:
                dyn_obs.initial_state = state_list[
                    idx_first_state
                ].convert_state_to_state(InitialState())
                if idx_first_state + 1 < len(state_list):
                    obs_trajectory = Trajectory(
                        state_list[idx_first_state + 1].time_step,
                        state_list[idx_first_state + 1 :],
                    )
                    dyn_obs.prediction = TrajectoryPrediction(
                        trajectory=obs_trajectory, shape=dyn_obs.prediction.shape
                    )
                else:
                    dyn_obs.prediction = None

    scenario.remove_obstacle(to_remove)

    # save file
    file_writer = CommonRoadFileWriter(
        scenario, planning_problem_set, file_format=FileFormat.XML
    )
    file_writer.write_to_file(save_path, OverwriteExistingFile.ALWAYS)
