# standard imports
from typing import Any, Optional
from dataclasses import dataclass, field

# ROS imports
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


@dataclass
class BaseSpec:
    """
    Base class for ROS interface specification.
    
    :var name: name (i.e., ROS topic) of the interface
    """

    # name (i.e., ROS topic) of the interface
    name: str = ""

    def __getitem__(self, item: str) -> Any:
        """
        Getter for base parameter value.
        
        :param item: item
        :return value: value
        :raises KeyError: if item is not a parameter of the class
        """
        try:
            value = self.__getattribute__(item)
        except AttributeError as e:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}") from e
        return value

    def __setitem__(self, key: str, value: Any) -> None:
        """
        Setter for item.
        
        :param key: key
        :param value: value
        :raises KeyError: if key is not a parameter of the class
        """
        try:
            self.__setattr__(key, value)
        except AttributeError as e:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}") from e


@dataclass
class QosSpec:
    """
    Class for ROS quality-of-service (QOS) specifications.

    Default values correspond to ROS2 defaults, see here: [About-Quality-of-Service-Settings](
    https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

    :var history: history policy
    :var reliability: reliability policy
    :var durability: durability policy
    """
    # history policy
    history: QoSHistoryPolicy = QoSHistoryPolicy.KEEP_LAST
    # reliability policy
    reliability: QoSReliabilityPolicy = QoSReliabilityPolicy.RELIABLE
    # durability_policy
    durability: QoSDurabilityPolicy = QoSDurabilityPolicy.VOLATILE


@dataclass
class PublisherSpec(BaseSpec):
    """
    Class for topic publisher specification
    
    :var msg_type: ROS/Autoware message type
    :var depth: depth
    :var qos_profile: QOS profile
    """
    # ROS/Autoware message type
    msg_type: Any = None
    # depth
    depth: int = 1
    # QOS profile
    qos_profile: QosSpec = field(default_factory=QosSpec)


@dataclass
class SubscriptionSpec(BaseSpec):
    """
    Class for topic subscription specification
    
    :var msg_type: ROS/Autoware message type
    :var depth: depth
    :var qos_profile: QOS profile
    """
    # ROS/Autoware message type
    msg_type: Any = None
    # depth
    depth: int = 1
    # QOS profile
    qos_profile: QosSpec = field(default_factory=QosSpec)


@dataclass
class SrvClientSpec(BaseSpec):
    """
    Class for service client specification
    
    :var srv_type: ROS/Autoware service type
    """
    # ROS/Autoware service type
    srv_type: Any = None
