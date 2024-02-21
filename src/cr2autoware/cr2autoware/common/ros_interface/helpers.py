from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


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
