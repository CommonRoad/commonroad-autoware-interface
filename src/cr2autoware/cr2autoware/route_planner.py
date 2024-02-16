# standard imports
from typing import Optional, Any, List
from abc import ABC, abstractmethod

# third party imports
import numpy as np

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# commonroad-io imports
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import PlanningProblem

# cr2autoware imports
import cr2autoware.utils as utils


class RoutePlannerInterface(ABC):
    """
    Abstract base class for a route planner interface.
    Defines basic attributes and abstract methods to be implemented by derived route planners.
    """

    # reference to route publisher
    _route_pub: Publisher
    # reference to ROS logger
    _logger: RcutilsLogger
    # verbose logging
    _verbose: bool

    def __init__(self, route_pub: Publisher, logger: RcutilsLogger, verbose: bool, lanelet_network: LaneletNetwork):
        """
        :param route_pub: ROS2 node publisher
        :param logger: ROS2 node logger
        :param verbose: verbose logging True/False
        :param lanelet_network: lanelet network from CommonRoad scenario
        """
        # initialize route publisher
        self._route_pub = route_pub
        # initialize ROS logger
        self._logger = logger
        # set logging verbosity
        self._verbose = verbose

        # CR lanelet network
        self.lanelet_network: LaneletNetwork = lanelet_network

        # initialize planner class (set in child class)
        self._planner: Any = self._initialize_planner()

        # reference path
        self._reference_path: Optional[np.ndarray] = None
        # list of route lanelet IDs
        self._route_list_lanelet_ids: Optional[List[int]] = None

        # bool route planned
        self._is_route_planned = False
        # bool published ref path
        self._is_ref_path_published = False

    @property
    def reference_path(self) -> Optional[np.ndarray]:
        """Getter for reference path"""
        return self._reference_path

    @property
    def is_route_planned(self) -> bool:
        """Getter for route planned bool"""
        return self._is_route_planned

    @property
    def is_ref_path_published(self) -> bool:
        """Getter for ref path published bool"""
        return self._is_ref_path_published

    @property
    def lanelet_network(self):
        """Getter for lanelet network"""
        return self._lanelet_network

    @lanelet_network.setter
    def lanelet_network(self, lln):
        """Setter for lanelet network"""
        self._lanelet_network = lln

    @abstractmethod
    def _initialize_planner(self, **kwargs):
        """Abstract method to initialize the self._planner."""
        pass

    @abstractmethod
    def _plan(self, planning_problem: PlanningProblem, **kwargs):
        """
        Plans a route and a reference path for the given planning problem.
        The planning algorithm is implemented in the respective planner (self._planner)
        """
        pass

    def plan(self, planning_problem: PlanningProblem, **kwargs):
        """
        Calls the encapsulated _plan function. If planning result is valid, sets _is_route_planned to True.
        """
        self._plan(planning_problem, **kwargs)

        if self._reference_path is not None and self._route_list_lanelet_ids:
            self._logger.info("<RoutePlannerInterface> valid route and reference path found")
            self._is_route_planned = True
        else:
            self._logger.info("<RoutePlannerInterface> No valid route and reference path found")
            self._is_route_planned = False

    def reset(self):
        """Resets route and reference path when desired by the user (e.g., upon pressing Clear Route)"""
        # reset reference path and route
        self._reference_path = None
        self._route_list_lanelet_ids = None

        self._is_route_planned = False
        # TODO publish empty reference path
        self.publish()

    def publish(self, **kwargs):
        """Publish route markers of planned reference path to visualize in RVIZ."""
        if self._reference_path is None:
            # publish empty reference path
            self._route_pub.publish([], [])

        # TODO implement functionality for publishing empty reference path
        self._route_pub.publish(utils.create_route_marker_msg(self._reference_path, **kwargs))
        self._is_ref_path_published = True
        if self._verbose:
            self._logger.info("<RoutePlannerInterface> Reference path published!")
            self._logger.info("<RoutePlannerInterface> Total reference path length: " + str(len(self._reference_path)))
