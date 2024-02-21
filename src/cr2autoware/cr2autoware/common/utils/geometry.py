# standard
from typing import List

# third party
import numpy as np

# ROS message imports
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion


def upsample_trajectory(traj: List[Point], dt_ratio: int):
    """Compute upsampled trajectory list.

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
    for i in range(
        1, dt_ratio - 1
    ):  # don't take first and last samples, they were already appended
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


def traj_linear_interpolate(point_1: Pose, point_2: Pose, smaller_dt: float, bigger_dt: float) -> Pose:
    """
    Interpolate a point between two points.
    :param point_1: point which will be smaller than interpolated point (on left-side)
    :param point_1: point which will be bigger than interpolated point (on right-side)
    :param smaller_dt: time step for the point will be interpolated
    :param bigger_dt: time step for the points which will be used for interpolation
    :return: pose of the interpolated point
    """
    new_point = Pose()
    new_point.position.x = point_1.position.x + (
        (point_2.position.x - point_1.position.x) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.position.y = point_1.position.y + (
        (point_2.position.y - point_1.position.y) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.position.z = point_1.position.z + (
        (point_2.position.z - point_1.position.z) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.x = point_1.orientation.x + (
        (point_2.orientation.x - point_1.orientation.x) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.y = point_1.orientation.y + (
        (point_2.orientation.y - point_1.orientation.y) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.z = point_1.orientation.z + (
        (point_2.orientation.z - point_1.orientation.z) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    new_point.orientation.w = point_1.orientation.w + (
        (point_2.orientation.w - point_1.orientation.w) / smaller_dt
    ) * (bigger_dt - smaller_dt)
    return new_point
