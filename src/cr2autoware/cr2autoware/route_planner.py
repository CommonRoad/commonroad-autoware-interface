from commonroad_dc.geometry.util import resample_polyline
from commonroad_route_planner.route_planner import RoutePlanner
import numpy as np
from scipy.interpolate import splev
from scipy.interpolate import splprep

import cr2autoware.utils as utils


class RoutePlannerInterface:
    """RoutePlannerInterface class represent the route planner."""

    def __init__(
        self,
        get_parameter,
        get_logger,
        scenario,
        route_pub,
    ):
        # ROS functions
        self.reference_path_published = False
        self.reference_path = None
        self.get_parameter = get_parameter
        self.get_logger = get_logger
        self.scenario = scenario
        self.route_pub = route_pub

    def plan(self, planning_problem):
        """Plan a route using commonroad route planner and the current scenario and planning problem."""
        self.reference_path_published = False

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger.info("Planning route")

        route_planner = RoutePlanner(self.scenario, planning_problem)
        reference_path = route_planner.plan_routes().retrieve_first_route().reference_path
        # smooth reference path
        tck, u = splprep(reference_path.T, u=None, k=3, s=0.0)
        u_new = np.linspace(u.min(), u.max(), 200)
        x_new, y_new = splev(u_new, tck, der=0)
        reference_path = np.array([x_new, y_new]).transpose()
        reference_path = resample_polyline(reference_path, 1)
        # remove duplicated vertices in reference path
        _, idx = np.unique(reference_path, axis=0, return_index=True)
        reference_path = reference_path[np.sort(idx)]
        self.reference_path = reference_path

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger.info("Route planning completed!")

    def _pub_route(self, path, velocities):
        """Publish planned route as marker to visualize in RVIZ."""
        self.reference_path_published = True
        self.route_pub.publish(utils.create_route_marker_msg(path, velocities))

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger.info("Reference path published!")
            self.get_logger.info("Path length: " + str(len(path)))
            self.get_logger.info("Velocities length: " + str(len(velocities)))
