# third party imports
import numpy as np
from scipy.interpolate import splev
from scipy.interpolate import splprep

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# commonroad-io imports
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import PlanningProblem

# commonroad-dc imports
from commonroad_dc.geometry.util import resample_polyline, chaikins_corner_cutting, compute_curvature_from_polyline

# commonroad-route-planner imports
from commonroad_route_planner.route_planner import RoutePlanner as CRRoutePlanner

# cr2autoware imports
from cr2autoware.route_planner import RoutePlannerInterface


class CommonRoadRoutePlanner(RoutePlannerInterface):
    """Interface for the CommonRoad Route Planner"""

    def __init__(self, route_pub: Publisher,
                 logger: RcutilsLogger,
                 verbose: bool,
                 lanelet_network: LaneletNetwork):

        super().__init__(route_pub=route_pub, logger=logger.get_child("route_planner"), verbose=verbose,
                         lanelet_network=lanelet_network)

        self._planner: CRRoutePlanner = self._initialize_planner()

    def _initialize_planner(self, **kwargs) -> CRRoutePlanner:
        """Implements abstract _initialize_planner from base class"""
        # TODO: initialize Route Planner properly
        # return CRRoutePlanner(lanelet_network=self.lanelet_network)
        pass

    def _plan(self, planning_problem: PlanningProblem, **kwargs):
        """Implements abstract plan method from base class"""
        self._is_ref_path_published = False

        # TODO don't initialize route planner object here
        self._planner = CRRoutePlanner(lanelet_network=self.lanelet_network, planning_problem=planning_problem)

        # TODO implement this function for updating planning problem in route planner
        # self._planner.set_planning_problem(planning_problem)

        if self._verbose:
            self._logger.info("<CommonRoadRoutePlanner>: Starting to plan route ...")

        try:
            planned_route = self._planner.plan_routes().retrieve_first_route()
        except IndexError:
            self._logger.info("<CommonRoadRoutePlanner>: No valid route could be found.")
            return

        self._route_list_lanelet_ids = planned_route.lanelet_ids
        self._reference_path = planned_route.reference_path

        # Postprocessing of reference path
        self._postprocess_ref_path()

        if self._verbose:
            self._logger.info("<CommonRoadRoutePlanner>: Route planning completed!")

    def _postprocess_ref_path(self, curvature_limit: float = 0.195, spline_smooth_fac: float = 5.0,
                              resampling_step: float = 1.0):
        """Postprocessing options of original reference path"""
        # reduce curvature simple
        ref_path_mod = self._simple_reduce_curvature(ref_path=self._reference_path, max_curv=curvature_limit)

        # spline smoothing
        tck, u = splprep(ref_path_mod.T, u=None, k=3, s=spline_smooth_fac)
        u_new = np.linspace(u.min(), u.max(), 200)
        x_new, y_new = splev(u_new, tck, der=0)
        ref_path_mod = np.array([x_new, y_new]).transpose()
        ref_path_mod = resample_polyline(ref_path_mod, resampling_step)

        # remove duplicated vertices in reference path
        _, idx = np.unique(ref_path_mod, axis=0, return_index=True)
        ref_path_mod = ref_path_mod[np.sort(idx)]

        self._reference_path = ref_path_mod

    @staticmethod
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