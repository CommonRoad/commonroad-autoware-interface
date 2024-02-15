# standard imports
from typing import Optional, Any, List
from abc import ABC, abstractmethod

# third party imports
import numpy as np
from scipy.interpolate import splev
from scipy.interpolate import splprep

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# commonroad-dc imports
from commonroad_dc.geometry.util import resample_polyline, chaikins_corner_cutting, compute_curvature_from_polyline

# commonroad-route-planner imports
from commonroad_route_planner.route_planner import RoutePlanner

# cr2autoware imports
import cr2autoware.utils as utils


# TODO: move to util functions
def _simple_reduce_curvature(ref_path: np.ndarray, max_curv: float, resample_step: float = 1.0):
        max_curvature = 0.5
        iter = 0
        while max_curvature > max_curv:
            ref_path = np.array(chaikins_corner_cutting(ref_path))
            ref_path = resample_polyline(ref_path, resample_step)
            abs_curvature = compute_curvature_from_polyline(ref_path)
            max_curvature = max(abs_curvature)
            iter += 1
        return ref_path


class RoutePlannerInterfaceABC(ABC):
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

    def __init__(self, route_pub: Publisher, logger: RcutilsLogger, verbose: bool):
        # initialize route publisher
        self._route_pub = route_pub
        # initialize ROS logger
        self._logger = logger
        # set logging verbosity
        self._verbose = verbose

        # initialize planner class (set in child class)
        self._planner: Any = None

        # reference path
        self._reference_path: Optional[np.ndarray] = None
        # list of route lanelet IDs
        self._route_list_lanelet_ids: Optional[List[int]] = None
        #



class RoutePlannerInterface:
    """RoutePlannerInterface class represent the route planner."""

    def __init__(
        self,
        verbose,
        get_logger,
        scenario,
        route_pub,
    ):
        # ROS functions
        self.reference_path_published = False
        self._reference_path = None
        self.verbose = verbose
        self.get_logger = get_logger
        self.scenario = scenario
        self._route_pub = route_pub

    @property
    def reference_path(self):
        """Getter for reference path"""
        return self._reference_path

    def plan(self, planning_problem, curvature_limit: float = 0.195, spline_smooth_fac: float = 5.0):
        """Plan a route using commonroad route planner and the current scenario and planning problem."""
        self.reference_path_published = False

        if self.verbose:
            self.get_logger.info("Planning route")

        route_planner = RoutePlanner(self.scenario, planning_problem)
        reference_path = route_planner.plan_routes().retrieve_first_route().reference_path

        # reduce curvature simple
        reference_path = _simple_reduce_curvature(ref_path=reference_path, max_curv=curvature_limit)

        # smooth reference path spline
        tck, u = splprep(reference_path.T, u=None, k=3, s=spline_smooth_fac)
        u_new = np.linspace(u.min(), u.max(), 200)
        x_new, y_new = splev(u_new, tck, der=0)
        reference_path = np.array([x_new, y_new]).transpose()
        reference_path = resample_polyline(reference_path, 1)

        # remove duplicated vertices in reference path
        _, idx = np.unique(reference_path, axis=0, return_index=True)
        reference_path = reference_path[np.sort(idx)]
        self._reference_path = reference_path

        if self.verbose:
            self.get_logger.info("Route planning completed!")

    def publish(self, path, velocities):
        """Publish planned route as marker to visualize in RVIZ."""
        self.reference_path_published = True
        self._route_pub.publish(utils.create_route_marker_msg(path, velocities))

        if self.verbose:
            self.get_logger.info("Reference path published!")
            self.get_logger.info("Path length: " + str(len(path)))
            self.get_logger.info("Velocities length: " + str(len(velocities)))
