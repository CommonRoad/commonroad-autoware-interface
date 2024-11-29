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

    -------------------
    **Handler Classes:**

    * [`DataGenerationHandler`](data_generation_handler.md)
    * [`EgoVehicleHandler`](ego_vehicle_handler.md)
    * [`PlanningProblemHandler`](planning_problem_handler.md)
    * [`ScenarioHandler`](scenario_handler.md)
    * [`SpotHandler`](spot_handler.md)

    -------------------
    **Methods:**
    
    * `_get_param(param_name:str)`: Helper function to get a ROS parameter value from self._node

    -------------------
    **Abstract Methods:**

    * `_init_parameters()`: Retrieve required ROS params from self._node
    * `_init_subscriptions()`: Initialize required subscribers for self._node
    * `_init_publishers()`: Initialize required publishers for self._node

    -------------------
    :var _node: reference to CR2Auto ROS2 node
    :var _logger: reference to ROS RcutilsLogger
    :var _VERBOSE: constant for verbose loggging
    """

    # reference to node
    _node: "Cr2Auto"
    # reference to ROS logger
    _logger: RcutilsLogger
    # verbose logging
    _VERBOSE: bool = False

    def __init__(self, node: "Cr2Auto", logger: RcutilsLogger, verbose: bool):
        """
        Constructor for BaseHandler class.

        :param node: reference to CR2Auto ROS2 node
        :param logger: reference to ROS RcutilsLogger
        :param verbose: flag for verbose logging
        """
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
        """
        Helper function to get a ROS parameter value from self._node
        
        :param param_name: name of the ROS parameter
        :return: value of the ROS parameter
        """
        return self._node.get_parameter(param_name).get_parameter_value()
