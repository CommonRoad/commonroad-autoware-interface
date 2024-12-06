# standard imports
from typing import List, Optional, Any
from abc import ABC, abstractmethod
import time

# third party
import numpy as np

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# Autoware imports
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_planning_msgs.msg import TrajectoryPoint  # type: ignore

# commonroad imports
from commonroad.scenario.state import TraceState
from commonroad.planning.goal import GoalRegion

# cr2autoware imports
from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState
from cr2autoware.common.utils.transform import orientation2quaternion
from cr2autoware.common.utils.transform import utm2map
from cr2autoware.common.utils.trajectory_utils import (
    get_closest_state,
    get_closest_idx
)
from cr2autoware.common.configuration import TrajectoryPlannerParams


class TrajectoryPlannerInterface(ABC):
    """
    Abstract base class for trajectory planner interface.

    Defines basic attributes and abstract methods which need to be implemented by all planner interfaces.

    :var _traj_pub: reference to trajectory publisher
    :var _logger: reference to ROS logger
    :var _verbose: constant for verbose logging
    :var _planner: reference to planner class
    :var _traj_planner_params: trajectory planner params from config
    :var _cr_state_list: planned trajectory as list of CommonRoad states
    :var _prev_state_list: previous trajectory as list of CommonRoad states
    """

    # reference to trajectory publisher
    _traj_pub: Publisher
    # reference to ROS logger
    _logger: RcutilsLogger
    # verbose logging
    _verbose: bool

    def __init__(
            self,
            traj_pub: Publisher,
            traj_planner_params: TrajectoryPlannerParams,
            logger: RcutilsLogger,
            verbose: bool
    ) -> None:
        """
        Constructor for TrajectoryPlannerInterface class.

        :param traj_pub: The ROS2 publisher to use which publishes the planned trajectory to Autoware
        :param logger: The ROS2 logger to use for logging
        :param verbose: Flag for verbose logging
        """
        # initialize trajectory publisher
        self._traj_pub = traj_pub
        # intialize ROS logger
        self._logger = logger
        # set logging verbosity
        self._verbose = verbose

        # trajectory planner parameters
        self._traj_planner_params = traj_planner_params

        # initialize planner class (set in child class)
        self._planner: Any = None

        # current CR trajectory state list
        self._cr_state_list: Optional[List[TraceState]] = None

        # store previous CR trajectory state list
        self._prev_state_list: Optional[List[TraceState]] = None

    @property
    def cr_state_list(self) -> Optional[List[TraceState]]:
        """
        Getter for CommonRoad state list.
        
        :return: List of CommonRoad states
        """
        return self._cr_state_list

    @abstractmethod
    def update(self, *args, **kwargs) -> None:
        """
        Updates all dynamic inputs of the planner for cyclic re-planning.
        
        :param args: Additional arguments
        :param kwargs: Additional keyword arguments
        """
        pass

    def plan(self, current_state: EgoVehicleState, goal: GoalRegion, **kwargs) -> None:
        """
        Calculates initial planner state for re-planning and plans a trajectory.
        
        :param current_state: Current ego vehicle state
        :param goal: Goal region
        :param kwargs: Additional keyword arguments
        """
        # Calculate initial planner state for re-
        t_start = time.perf_counter()
        if self._traj_planner_params.replan_always_from_measured:
            self._logger.debug("Re-planning from MEASURED state.")
            init_state = current_state
        else:
            init_state = self._calc_init_planner_state(current_state)

        # timer for init state update
        t_elapsed = time.perf_counter() - t_start
        self._logger.debug(f"\t Planner init state calculation took: {t_elapsed} s")

        # call planner
        self._plan(init_state, goal, **kwargs)

    @abstractmethod
    def _plan(self, init_state: EgoVehicleState, goal: GoalRegion, **kwargs) -> None:
        """
        Abstract method for planning trajectory.
        The planning algorithm is implemented in the respective planner (self._planner).

        :param init_state: Initial state for planning
        :param goal: Goal region
        :param kwargs: Additional keyword arguments
        """
        pass

    def _calc_init_planner_state(self, current_state: EgoVehicleState) -> EgoVehicleState:
        """
        Calculates the initial planner state for re-planning based the on current measured state and the trajectory
        from the previous cycle.

        :param current_state: Current measured ego vehicle state
        :param vel_threshold: velocity deviation threshold for re-planning
        """
        if self._prev_state_list is not None:
            # deviation thresholds
            vel_threshold = self._traj_planner_params.replan_vel_deviation
            dist_threshold = self._traj_planner_params.replan_dist_deviation

            # get closest idx for position on previous trajectory
            pos_array = np.array([state.position for state in self._prev_state_list])
            closest_idx, closest_dist = get_closest_idx(pos_array, current_state.position)

            # if distance deviation too large: re-plan from measured state
            if closest_dist > dist_threshold:
                self._logger.debug("Re-planning from MEASURED state.")
                return current_state

            # get closest interpolated state on previous trajectory
            interp_state = get_closest_state(
                traj=self._prev_state_list,
                current_state=current_state,
                closest_idx=closest_idx
            )

            # if velocity deviation too large: use measured velocity for re-planning
            if np.abs(interp_state.velocity - current_state.velocity) > vel_threshold:
                interp_state.velocity = current_state.velocity

            # return interpolated state
            self._logger.debug("Re-planning from PLANNED state.")
            return interp_state
        else:
            return current_state


    def _prepare_trajectory_msg(self, origin_transformation: List, elevation: float) -> AWTrajectory:
        """
        Converts the CommonRoad state list into a AWTrajectory message type for publishing to Autoware.

        Publishes the trajectory starting from the second state in the state list. The first state (index 0) is
        simply the current initial state. If no trajectory is planned, an empty trajectory is published.

        :param origin_transformation: Origin transformation for UTM to map transformation
        :param elevation: Elevation for the trajectory
        :return: AWTrajectory message
        """
        if self._verbose:
            self._logger.info("Preparing trajectory message!")

        # AW Trajectory message
        aw_traj = AWTrajectory()
        aw_traj.header.frame_id = "map"

        # Publish empty trajectory if nothing planned
        if not self.cr_state_list:
            self._logger.info("New empty trajectory published !!!")
            return aw_traj

        # Convert CR Trajectory to AW Trajectory message
        # We publish the trajectory starting from the second state in the state list. The first state (index 0) is
        # simply the current initial state
        position_list = []
        for i in range(1, len(self._cr_state_list)):
            cr_state = self._cr_state_list[i]

            # transform position
            new_point = TrajectoryPoint()
            new_point.pose.position = utm2map(origin_transformation, cr_state.position)

            # Post process trajectory elevation (z coordinate)
            new_point.pose.position.z = elevation
            position_list.append([cr_state.position[0], cr_state.position[1], elevation])
            new_point.pose.orientation = orientation2quaternion(cr_state.orientation)
            new_point.longitudinal_velocity_mps = float(cr_state.velocity)

            # front_wheel_angle_rad not given by Autoware planner
            # new_point.front_wheel_angle_rad = states[i].steering_angle
            new_point.acceleration_mps2 = float(cr_state.acceleration)
            aw_traj.points.append(new_point)

        return aw_traj

    def publish(self, origin_transformation: List, elevation: float) -> None:
        """
        Publishes the output trajectory as AWTrajectory message type.
        
        :param origin_transformation: Origin transformation for UTM to map transformation
        :param elevation: Elevation for the trajectory
        """
        traj_msg = self._prepare_trajectory_msg(origin_transformation, elevation)

        self._traj_pub.publish(traj_msg)

        if self._verbose:
            self._logger.info("New trajectory published !!!")
