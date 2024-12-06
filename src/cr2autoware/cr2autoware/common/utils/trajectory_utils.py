# standard
from typing import List, Tuple, Optional

# third party
import numpy as np

# commonroad
from commonroad.scenario.state import TraceState
from commonroad.common.util import make_valid_orientation

# cr2autoware
from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState


def _lerp(val1: float, val2: float, ratio: float):
    """Linear interpolation between two scalar values"""
    return val1 + (val2 - val1) * ratio


def get_closest_idx(
        position_array: np.ndarray,
        target_pos: np.ndarray,
) -> Tuple[int, float]:
    """
    Calculates closest index to an array of points for a given target position via Euclidean distance
    Returns Tuple of index and distance

    :param position_array: Array of 2d positions
    :param target_pos: 2d Target position
    """
    dist_array = np.linalg.norm(position_array - target_pos, axis=1)
    closest_index = np.argmin(dist_array)
    return (closest_index, dist_array[closest_index])


def get_closest_state(
        traj: List[TraceState],
        current_state: EgoVehicleState,
        closest_idx: Optional[int] = None,
) -> EgoVehicleState:
    """
    Calculates the closest state on a given trajectory for a provided state via interpolation.

    :param traj: A trajectory as a list of states
    :param current_state: Provided state for which the closest state on traj should be calculated
    :param closest_idx: [Optional] closest index of point if computed beforehand
    """
    # get closest idx on previous trajectory
    if closest_idx is None:
        pos_array = np.array([state.position for state in traj])
        closest_idx, closest_dist = get_closest_idx(pos_array, current_state.position)

    # closest point
    closest_pt = traj[closest_idx]

    # get segment on trajectory
    if closest_idx == 0:
        curr_pt = closest_pt
        next_pt = traj[closest_idx + 1]
    elif closest_idx == len(traj) - 1:
        curr_pt = traj[closest_idx - 1]
        next_pt = closest_pt
    else:
        dist_next = np.linalg.norm(
            traj[closest_idx + 1].position - current_state.position
        )
        dist_prev = np.linalg.norm(
            traj[closest_idx - 1].position - current_state.position
        )
        if dist_next < dist_prev:
            curr_pt = closest_pt
            next_pt = traj[closest_idx + 1]
        else:
            curr_pt = traj[closest_idx - 1]
            next_pt = closest_pt

    # interpolation ratio
    vec1 = next_pt.position - curr_pt.position
    vec2 = current_state.position - curr_pt.position
    len_vec1 = np.dot(vec1, vec1)
    ratio = np.dot(vec1, vec2) / len_vec1 if len_vec1 >= 1e-3 else 0.0
    clamped_ratio = np.clip(ratio, 0, 1)

    # position
    pos_interp = curr_pt.position + clamped_ratio * vec1
    # velocity
    vel_interp = _lerp(curr_pt.velocity, next_pt.velocity, clamped_ratio)
    # acceleration
    acc_interp = _lerp(curr_pt.acceleration, next_pt.acceleration, clamped_ratio)
    # orientation
    theta_interp = make_valid_orientation(
        _lerp(curr_pt.orientation, next_pt.orientation, clamped_ratio)
    )
    # yaw_rate
    yaw_rate_interp = _lerp(curr_pt.yaw_rate, next_pt.yaw_rate, clamped_ratio)
    # steering angle
    steering_angle_interp = _lerp(curr_pt.steering_angle, next_pt.steering_angle, clamped_ratio)

    # make output
    return EgoVehicleState(
        position=pos_interp,
        orientation=theta_interp,
        velocity=vel_interp,
        yaw_rate=yaw_rate_interp,
        acceleration=acc_interp,
        slip_angle=0.0,
        steering_angle=steering_angle_interp,
        time_step=current_state.time_step
    )
