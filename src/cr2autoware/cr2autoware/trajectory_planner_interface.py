# standard imports
import typing
from typing import List, Optional, Any
from abc import ABC
from abc import abstractmethod

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# Autoware imports
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory

# commonroad imports
from commonroad.scenario.state import TraceState
from commonroad.planning.goal import GoalRegion

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


class TrajectoryPlannerInterface(ABC):
    """
    Abstract base class for trajectory planner interface.
    Defines basic attributes and abstract methods which need to be implemented by all planner interfaces
    """

    # reference to trajectory publisher
    _traj_pub: Publisher
    # reference to ROS logger
    _logger: RcutilsLogger

    def __init__(self, traj_pub: Publisher, logger: RcutilsLogger):
        """
        :param traj_pub: The ROS2 publisher to use which publishes the planned trajectory to Autoware
        """
        # initialize trajectory publisher
        self._traj_pub = traj_pub

        # intialize ROS logger
        self._logger = logger

        # initialize planner class (set in child class)
        self._planner: Any = None

        # initialize CR trajectory state list
        self._cr_state_list: Optional[List[TraceState]] = None

    @property
    def cr_state_list(self) -> Optional[List[TraceState]]:
        return self._cr_state_list

    @abstractmethod
    def update(self, *args, **kwargs):
        """Updates all dynamic inputs of the planner for cyclic re-planning"""
        pass

    @abstractmethod
    def plan(self, current_state: TraceState, goal: GoalRegion, **kwargs):
        """Plans a trajectory. The planning algorithm is implemented in the respective planner (self._planner)"""
        pass

    @abstractmethod
    def _prepare_trajectory_msg(self) -> AWTrajectory:
        """Converts the CommonRoad state list into a AWTrajectory message type for publishing to Autoware"""
        pass

    def publish_trajectory_msg(self, traj_msg: AWTrajectory):
        """Publishes the output trajectory as AWTrajectory message type"""
        self._traj_pub.publish(traj_msg)
