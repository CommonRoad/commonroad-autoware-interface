import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.util import FileFormat
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import InitialState, CustomState
from commonroad.scenario.trajectory import Trajectory


def interpolate_obstacles(
    scenario_path: str,
    save_path: str,
) -> None:

    scenario, planning_problem_set = CommonRoadFileReader(
        filename_2020a=scenario_path
    ).open()

    for obs in scenario.dynamic_obstacles:
        interpolate_trajectory(obs, scenario.dt)

    file_writer = CommonRoadFileWriter(scenario, planning_problem_set, file_format=FileFormat.XML)
    file_writer.write_to_file(save_path, overwrite_existing_file=OverwriteExistingFile.ALWAYS)


def interpolate_trajectory(
    obs: DynamicObstacle,
    dt: float,
) -> None:
    """
    Interpolates the trajectory of a dynamic obstacle.
    :param obs: Dynamic obstacle to interpolate.
    :param dt: Time step size of scenario.
    """

    if obs.prediction is None:
        return

    # Fix velocity and acceleration values by differentiating the position
    last_pos = obs.initial_state.position
    last_velocity = obs.initial_state.velocity
    for state in obs.prediction.trajectory.state_list:
        state.velocity = np.linalg.norm(last_pos - state.position) / dt
        state.acceleration = (last_velocity - state.velocity) / dt
        last_pos = state.position
        last_velocity = state.velocity

    states = np.array([
        [state.time_step, state.position[0], state.position[1], state.orientation, state.velocity]
        for state in sorted([obs.initial_state] + obs.prediction.trajectory.state_list, key=lambda x: x.time_step)
    ])
    time_steps = np.arange(
        obs.initial_state.time_step,
        obs.prediction.final_time_step + 1,
        dtype=int
    )
    interpolated = np.array([np.interp(time_steps, states[:, 0], states[:, i]) for i in range(1, 5)])
    interpolated = np.vstack((time_steps, interpolated)).T
    interpolated_states = [
        CustomState(
            time_step=int(interp[0]),
            position=interp[1:3],
            orientation=interp[3],
            velocity=interp[4],
        )
        for interp in interpolated
    ]

    obs.initial_state = interpolated_states[0].convert_state_to_state(InitialState())
    if len(interpolated_states) > 1:
        resampled_traj = Trajectory(
            initial_time_step=interpolated_states[1].time_step,
            state_list=interpolated_states[1:],
        )
        obs.prediction = TrajectoryPrediction(resampled_traj, obs.prediction.shape)
    else:
        obs.prediction = None
