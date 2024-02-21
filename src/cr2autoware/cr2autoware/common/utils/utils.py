# standard imports
import enum
import math
import pathlib
from typing import List

# third party imports
import matplotlib.pyplot as plt
import numpy as np

# ROS imports
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from ament_index_python import get_package_share_directory
import rclpy.logging as ros_logging

# comonroad-io imports
from commonroad.common.util import AngleInterval
from commonroad.geometry.shape import Circle
from commonroad.geometry.shape import Polygon
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import AngleExactOrInterval
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer

# Autoware message imports
from dummy_perception_publisher.msg import Object

# ROS message imports
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

logger = ros_logging.get_logger(__name__)



# _process_dynamic_obs helper method


# TODO fix import -> move to ros_interface.helpers.py
def create_qos_profile(history_policy: QoSHistoryPolicy,
                       reliability_policy: QoSReliabilityPolicy,
                       durability_policy: QoSDurabilityPolicy,
                       depth=1) -> QoSProfile:
    """Creates a ROS Quality-of-Service (QOS) profile with the specified settings"""
    qos_profile = QoSProfile(depth=depth)
    qos_profile.history = history_policy
    qos_profile.reliability = reliability_policy
    qos_profile.durability = durability_policy

    return qos_profile
