"""
Transform Utils
===========================

This module contains utility functions for transforming data between CommonRoad and Autoware.

---------------------------
"""

# standard imports
import math

# third party
import numpy as np

# ROS message imports
from geometry_msgs.msg import Quaternion  # type: ignore
from geometry_msgs.msg import Point  # type: ignore

# commonroad-io imports
from commonroad.scenario.state import AngleExactOrInterval
from commonroad.common.util import AngleInterval


def orientation2quaternion(orientation: AngleExactOrInterval) -> Quaternion:
    """
    Transform orientation (in CommonRoad) to quaternion (in Autoware).

    :param orientation: orientation angles
    :return: orientation quaternion
    """
    if isinstance(orientation, AngleInterval):
        raise NotImplementedError("orientation2quaternion not implemented for type AngleInterval")
    quat = Quaternion()
    quat.w = math.cos(orientation * 0.5)
    quat.z = math.sin(orientation * 0.5)
    return quat


def quaternion2orientation(quaternion: Quaternion) -> float:
    """
    Transform quaternion (in Autoware) to orientation (in CommonRoad).

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


def map2utm(origin_transformation, p: Point) -> np.ndarray:
    """
    Transform position (in Autoware) to position (in CommonRoad).

    :param origin_transformation: list or array with 2 elements
    :param p: position Autoware
    :return: position CommonRoad
    """
    _x = origin_transformation[0] + p.x
    _y = origin_transformation[1] + p.y
    return np.array([_x, _y])


def utm2map(origin_transformation, position: np.array) -> Point:
    """
    Transform position (in CommonRoad) to position (in Autoware).

    :param origin_transformation: list or array with 2 elements
    :param position: position CommonRoad
    :return: position Autoware
    """
    p = Point()
    p.x = position[0] - origin_transformation[0]
    p.y = position[1] - origin_transformation[1]
    return p
