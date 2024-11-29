"""
ROS Interface Creation Module
=============================

This module provides functions to create ROS publishers, subscribers, and service clients with the
specified settings for the CR2Autoware node.

The following functions are provided::

    - create_publisher(node: Node, spec: PublisherSpec) -> Publisher

    - create_subscription(node: Node, spec: SubscriptionSpec, callback_func: Callable,
                            callback_group: Type[CallbackGroup] = None) -> Subscription

    - create_client(node: Node, spec: SrvClientSpec) -> Client

    - create_qos_profile(history_policy: QoSHistoryPolicy,
                            reliability_policy: QoSReliabilityPolicy,
                            durability_policy: QoSDurabilityPolicy,
                            depth=1) -> QoSProfile
"""
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
    """
    Creates a topic publisher with the given specification for the node.
    
    :param node: The ROS2 node to create the publisher for.
    :param spec: The specification of the publisher to create.
    :return: The created publisher.
    """
    _qos_pofile: QoSProfile = create_qos_profile(spec.qos_profile.history,
                                                 spec.qos_profile.reliability,
                                                 spec.qos_profile.durability,
                                                 spec.depth)
    _pub: Publisher = node.create_publisher(spec.msg_type,
                                            spec.name,
                                            _qos_pofile)
    return _pub


def create_subscription(node: Node, spec: SubscriptionSpec, callback_func: Callable,
                        callback_group: Type[CallbackGroup] = None) -> Subscription:
    """
    Creates a topic subscription with the given specification for the node.
    
    :param node: The ROS2 node to create the subscription for.
    :param spec: The specification of the subscription to create.
    :param callback_func: The callback function to be called when a message is received.
    :param callback_group: The callback group to associate with the subscription.
    :return: The created subscription.
    """
    _sub: Subscription = node.create_subscription(spec.msg_type,
                                                  spec.name,
                                                  callback_func,
                                                  spec.depth,
                                                  callback_group=callback_group)
    return _sub


def create_client(node: Node, spec: SrvClientSpec) -> Client:
    """
    Creates a service client with the given specification for the node.
    
    :param node: The ROS2 node to create the service client for.
    :param spec: The specification of the service client to create.
    :return: The created service client.
    """
    _client = node.create_client(spec.srv_type,
                                 spec.name)
    return _client


def create_qos_profile(history_policy: QoSHistoryPolicy,
                       reliability_policy: QoSReliabilityPolicy,
                       durability_policy: QoSDurabilityPolicy,
                       depth=1) -> QoSProfile:
    """
    Creates a ROS Quality-of-Service (QOS) profile with the specified settings.
    
    :param history_policy: The history policy to use.
    :param reliability_policy: The reliability policy to use.
    :param durability_policy: The durability policy to use.
    :param depth: The depth of the QOS profile.
    :return: The created QOS profile.
    """
    qos_profile = QoSProfile(depth=depth)
    qos_profile.history = history_policy
    qos_profile.reliability = reliability_policy
    qos_profile.durability = durability_policy
    return qos_profile
