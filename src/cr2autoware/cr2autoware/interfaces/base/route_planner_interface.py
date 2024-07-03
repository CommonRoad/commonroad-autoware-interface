# standard imports
from typing import Optional, Any, List
from abc import ABC, abstractmethod

# third party imports
import numpy as np

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# ROS msgs
from visualization_msgs.msg import MarkerArray

# commonroad-io imports
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import PlanningProblem

# commonroad-dc imports
from commonroad_dc.geometry.util import compute_pathlength_from_polyline

# cr2autoware imports
from cr2autoware.common.utils.message import create_route_marker_msg


class RoutePlannerInterface(ABC):
    """
    Abstract base class for a route planner interface.

    Defines basic attributes and abstract methods to be implemented by derived route planners.

    :var _route_pub: reference to route publisher
    :var _logger: reference to ROS logger
    :var _verbose: constant for verbose logging
    :var lanelet_network: lanelet network from CommonRoad scenario
    :var _planner: reference to planner class
    :var _reference_path: reference path
    :var _route_list_lanelet_ids: list of route lanelet IDs
    :var _is_route_planned: bool to check if route planned
    :var _is_ref_path_published: bool to check if reference path published
    """

    # reference to route publisher
    _route_pub: Publisher
    # reference to ROS logger
    _logger: RcutilsLogger
    # verbose logging
    _verbose: bool

    def __init__(self, route_pub: Publisher, logger: RcutilsLogger, verbose: bool, lanelet_network: LaneletNetwork,
                 **kwargs):
        """
        Constructor for RoutePlannerInterface class.

        :param route_pub: ROS2 node publisher for route
        :param logger: ROS2 node logger
        :param verbose: verbose logging True/False
        :param lanelet_network: lanelet network from CommonRoad scenario
        :param kwargs: additional keyword arguments
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
        self._planner: Any = self._initialize_planner(**kwargs)

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
        """
        Getter for reference path.
        
        :return: reference path
        """
        return self._reference_path

    @property
    def is_route_planned(self) -> bool:
        """
        Getter for route planned bool.
        
        :return: route planned bool
        """
        return self._is_route_planned

    @property
    def is_ref_path_published(self) -> bool:
        """
        Getter for ref path published bool.
        
        :return: ref path published bool
        """
        return self._is_ref_path_published

    @property
    def lanelet_network(self) -> LaneletNetwork:
        """
        Getter for lanelet network.
        
        :return: lanelet network
        """
        return self._lanelet_network

    @lanelet_network.setter
    def lanelet_network(self, lln):
        """Setter for lanelet network"""
        self._lanelet_network = lln

    @abstractmethod
    def _initialize_planner(self, **kwargs) -> Any:
        """
        Abstract method to initialize the self._planner.
        
        :param kwargs: additional keyword arguments
        :return: planner class
        """
        pass

    @abstractmethod
    def _plan(self, planning_problem: Optional[PlanningProblem] = None, **kwargs) -> None:
        """
        Plans a route and a reference path for the given planning problem.

        The planning algorithm is implemented in the respective planner (self._planner).

        :param planning_problem: planning problem
        """
        pass

    def plan(self, planning_problem: Optional[PlanningProblem] = None, **kwargs) -> None:
        """
        Calls the encapsulated _plan function. If planning result is valid, sets _is_route_planned to True.

        :param planning_problem: planning problem
        :param kwargs: additional keyword arguments
        """
        self.reset()
        self._logger.info("<RoutePlannerInterface>: Planning route")
        self._plan(planning_problem, **kwargs)

        if self._reference_path is not None and self._route_list_lanelet_ids:
            self._logger.info("<RoutePlannerInterface> valid route and reference path found")
            self._is_route_planned = True
        else:
            self._logger.info("<RoutePlannerInterface> No valid route and reference path found")
            self._is_route_planned = False

    def reset(self) -> None:
        """Resets route and reference path (e.g., when desired by the user upon pressing Clear Route)."""
        # reset reference path and route
        self._reference_path = None
        self._route_list_lanelet_ids = None

        self._is_route_planned = False
        # publish empty reference path
        self.publish()

    @staticmethod
    def _prepare_route_marker_msg(path: np.ndarray = None, velocities: np.ndarray = None,
                                  elevation: Optional[float] = None) -> MarkerArray:
        """
        Prepare route marker message for visualization in RVIZ.

        :param path: reference path
        :param velocities: velocities along the reference path
        :param elevation: elevation of the reference path
        :return: route marker message
        """
        if path is None:
            path = np.array([])
        if velocities is None:
            velocities = np.array([])

        # postprocess elevation of reference path
        elevation = elevation if elevation is not None else 0.0
        
        return create_route_marker_msg(path, velocities, elevation)

    def publish(self, path: np.ndarray = None, velocities: np.ndarray = None, elevation: float = None) -> None:
        """
        Publish route markers of planned reference path to visualize in RVIZ.

        :param path: reference path
        :param velocities: velocities along the reference path
        :param elevation: elevation of the reference path
        """
        route_marker_msg = self._prepare_route_marker_msg(path, velocities, elevation)
        self._route_pub.publish(route_marker_msg)

        if self.is_route_planned:
            self._is_ref_path_published = True
            if self._verbose:
                self._logger.info("<RoutePlannerInterface> Reference path published!")
                self._logger.info("<RoutePlannerInterface> Total reference path length: " +
                                  str(compute_pathlength_from_polyline(self._reference_path)))
        else:
            # publish empty reference path
            if self._verbose:
                self._logger.info("<RoutePlannerInterface> Empty reference path published!")
