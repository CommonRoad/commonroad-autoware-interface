# standard imports
import typing
from abc import ABC, abstractmethod

# ROS imports
from rclpy.impl.rcutils_logger import RcutilsLogger
from rcl_interfaces.msg import ParameterValue
from rclpy.logging import LoggingSeverity

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


class BaseHandler(ABC):
    """
    Abstract base class for handler classes.
    Defines basic attributes and methods which need to be implemented in all handler classes.

    ======== class variables:
    _node: reference to CR2Auto ROS2 node
    _logger: reference to ROS RcutilsLogger
    _VERBOSE: constant for verbose loggging

    ======== methods:
    _get_param(): get a ROS parameter value from _node

    ======== abstract methods:
    _init_parameters(): retrieve required ROS params from node
    _init_subscriptions(): initialize required subscribers
    _init_publishers(): initialize required publishers
    """

    # reference to node
    _node: "Cr2Auto"
    # reference to ROS logger
    _logger: RcutilsLogger
    # verbose logging
    _VERBOSE: bool = False

    def __init__(self, node: "Cr2Auto", logger: RcutilsLogger, verbose: bool):
        self._node = node
        self._logger = logger
        self._VERBOSE = verbose

        if self._VERBOSE:
            self._logger.set_level(LoggingSeverity.DEBUG)

    @abstractmethod
    def _init_parameters(self) -> None:
        """Retrieve required ROS params from self._node"""
        pass

    @abstractmethod
    def _init_subscriptions(self) -> None:
        """Initialize required subscribers for self._node"""
        pass

    @abstractmethod
    def _init_publishers(self) -> None:
        """Initialize required publishers for self._node"""
        pass

    def _get_param(self, param_name:str) -> ParameterValue:
        """Helper function to get a ROS parameter value from self._node"""
        return self._node.get_parameter(param_name).get_parameter_value()
