# standard imports
from typing import Callable, Type

# ROS imports
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# cr2autoware imports
from .specs_base import PublisherSpec
from .specs_base import SubscriptionSpec
from .specs_base import SrvClientSpec


def create_publisher(node: Node, spec: PublisherSpec) -> Publisher:
    """Creates a topic publisher with the given specification for the node"""
    return


def create_subscription(node: Node, spec: SubscriptionSpec, callback_func: Callable,
                        callback_group: Type[CallbackGroup] = None) -> Subscription:
    """Creates a topic subscription with the given specification for the node"""
    return


def create_client(node: Node, spec: SrvClientSpec) -> Client:
    """Creates a service client with the given specification for the node"""
    return


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
