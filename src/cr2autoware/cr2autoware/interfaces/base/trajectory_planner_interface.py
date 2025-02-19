# standard imports
from typing import List, Optional, Any
from abc import ABC, abstractmethod

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


class TrajectoryPlannerInterface(ABC):
    """
    Abstract base class for trajectory planner interface.

    Defines basic attributes and abstract methods which need to be implemented by all planner interfaces.

    :var _traj_pub: reference to trajectory publisher
    :var _logger: reference to ROS logger
    :var _verbose: constant for verbose logging
    :var _planner: reference to planner class
    :var _cr_state_list: list of CommonRoad states
    """

    # reference to trajectory publisher
    _traj_pub: Publisher
    # reference to ROS logger
    _logger: RcutilsLogger
    # verbose logging
    _verbose: bool

    def __init__(self, traj_pub: Publisher, logger: RcutilsLogger, verbose: bool):
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

        # initialize planner class (set in child class)
        self._planner: Any = None

        # initialize CR trajectory state list
        self._cr_state_list: Optional[List[TraceState]] = None

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

    @abstractmethod
    def plan(self, current_state: EgoVehicleState, goal: GoalRegion, **kwargs) -> None:
        """
        Plans a trajectory. The planning algorithm is implemented in the respective planner (self._planner).
        
        :param current_state: Current ego vehicle state
        :param goal: Goal region
        :param kwargs: Additional keyword arguments
        """
        pass

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
