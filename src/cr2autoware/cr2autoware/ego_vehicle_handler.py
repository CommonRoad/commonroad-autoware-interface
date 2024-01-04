# standard imports
import os
import typing
from typing import Any
from typing import Dict
from typing import Optional

# third party imports
import numpy as np

# ROS message imports
from geometry_msgs.msg import PoseStamped

# ROS imports
import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher
from rclpy.logging import LoggingSeverity

# commonroad imports
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.state import CustomState

# cr2autoware imports
import cr2autoware.utils as utils

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


class EgoVehicleHandler:
    """
    # Update of the CR Ego vehicle with current vehicle state from Autoware.

    # This class should contain all methods related to updating the CommonRoad Ego Vehicle object with the current ego state from Autoware
    # current ego vehicle is a class attribute (e.g., self.ego_vehicle)
    # contains subscription/callbacks to current ego vehicle state from AW
    # contains all associated methods necessary for transformations etc...
    # outward interface to retrieve current ego vehicle: e.g., EgoVehicleHandler.get_ego_vehicle().
    """

    # Constants and parameters
    VERBOSE: bool = False

    _logger: RcutilsLogger

    _ego_vehicle = None
    _ego_vehicle_state: Optional[CustomState] = None
    _current_vehicle_state = None
    _node: "Cr2Auto"
    _last_msg: Dict[str, Any] = {}
    _vehicle_length: Optional[float] = None
    _vehicle_width: Optional[float] = None
    _vehicle_wheelbase: Optional[float] = None
    _vehicle_wb_front_axle: Optional[float] = None
    _vehicle_wb_rear_axle: Optional[float] = None
    _vehicle_max_steer_angle: Optional[float] = None
    _vehicle_max_acceleration: Optional[float] = None

    # Publishers
    _OBSTACLE_PUBLISHER: Publisher

    @property
    def ego_vehicle(self):
        return self._ego_vehicle

    # TODO remove this setter after fixing self.create_ego_with_cur_loacation()
    @ego_vehicle.setter
    def ego_vehicle(self, ego_vehicle) -> None:
        self._ego_vehicle = ego_vehicle

    @property
    def ego_vehicle_state(self) -> Optional[CustomState]:
        return self._ego_vehicle_state

    @property
    def current_vehicle_state(self):
        return self._current_vehicle_state

    # TODO: remove this setter after moving subscriber and callback to this class from CR2Autoware
    @current_vehicle_state.setter
    def current_vehicle_state(self, current_vehicle_state) -> None:
        self._current_vehicle_state = current_vehicle_state

    @property
    def vehicle_length(self):
        return self._vehicle_length

    @property
    def vehicle_width(self):
        return self._vehicle_width

    @property
    def vehicle_wheelbase(self):
        return self._vehicle_wheelbase

    @property
    def vehicle_wb_front_axle(self):
        return self._vehicle_wb_front_axle

    @property
    def vehicle_wb_rear_axle(self):
        return self._vehicle_wb_rear_axle

    @property
    def vehicle_max_steer_angle(self):
        return self._vehicle_max_steer_angle

    @property
    def vehicle_max_acceleration(self):
        return self._vehicle_max_acceleration

    def __init__(self, node: "Cr2Auto"):
        self._node = node
        self._logger = node.get_logger().get_child("ego_vehicle_handler")

        self._ego_vehicle = None
        self._ego_vehicle_state = None
        self._current_vehicle_state = None
        self._last_msg = {}

        # set logging verbosity
        self.VERBOSE = self._node.get_parameter("general.detailed_log").get_parameter_value().bool_value
        if self.VERBOSE:
            self._logger.set_level(LoggingSeverity.DEBUG)

        # create ego vehicle dimension infos
        self._create_ego_vehicle_info()

    def process_current_state(self) -> None:
        """Calculate the current commonroad state from the autoware latest state message."""
        if self._current_vehicle_state is not None:
            source_frame = self._current_vehicle_state.header.frame_id
            time_step = 0
            # lookup transform
            succeed = self._node.tf_buffer.can_transform(
                "map",
                source_frame,
                rclpy.time.Time(),
            )
            if not succeed:
                self._logger.error(f"Failed to transform from {source_frame} to map frame")
                return None

            if source_frame != "map":
                temp_pose_stamped = PoseStamped()
                temp_pose_stamped.header = self._current_vehicle_state.header
                temp_pose_stamped.pose = self._current_vehicle_state.pose.pose
                pose_transformed = self.transform_pose(temp_pose_stamped, "map")
                position = utils.map2utm(
                    self._node.origin_transformation, pose_transformed.pose.position
                )
                orientation = utils.quaternion2orientation(pose_transformed.pose.orientation)
            else:
                position = utils.map2utm(
                    self._node.origin_transformation,
                    self._current_vehicle_state.pose.pose.position,
                )
                orientation = utils.quaternion2orientation(
                    self._current_vehicle_state.pose.pose.orientation
                )
            # steering_angle=  arctan2(wheelbase * yaw_rate, velocity)
            steering_angle = np.arctan2(
                self.vehicle_wheelbase * self._current_vehicle_state.twist.twist.angular.z,
                self._current_vehicle_state.twist.twist.linear.x,
            )

            self._ego_vehicle_state = CustomState(
                position=position,
                orientation=orientation,
                velocity=self._current_vehicle_state.twist.twist.linear.x,
                yaw_rate=self._current_vehicle_state.twist.twist.angular.z,
                slip_angle=0.0,
                time_step=time_step,
                steering_angle=steering_angle,
            )

    def update_ego_vehicle(self):
        """Update the commonroad scenario with the latest vehicle state and obstacle messages received."""
        # process last state message
        if self._current_vehicle_state is not None:
            self.process_current_state()
        else:
            if self._node.get_parameter("general.detailed_log").get_parameter_value().bool_value:
                self._node.get_logger().info("has not received a vehicle state yet!")
            return

    def _create_ego_vehicle_info(self):
        """Set the dimensions and kinematic parameters of the ego vehicle."""
        # get parameters from node
        wheel_base = (
            self._node.get_parameter("vehicle.wheel_base").get_parameter_value().double_value)
        wheel_tread = (
            self._node.get_parameter("vehicle.wheel_tread").get_parameter_value().double_value)
        wb_front_axle = (
            self._node.get_parameter("vehicle.wb_front_axle").get_parameter_value().double_value)
        wb_rear_axle = (
            self._node.get_parameter("vehicle.wb_rear_axle").get_parameter_value().double_value)
        front_overhang = (
            self._node.get_parameter("vehicle.front_overhang").get_parameter_value().double_value)
        rear_overhang = (
            self._node.get_parameter("vehicle.rear_overhang").get_parameter_value().double_value)
        left_overhang = (
            self._node.get_parameter("vehicle.left_overhang").get_parameter_value().double_value)
        right_overhang = (
            self._node.get_parameter("vehicle.right_overhang").get_parameter_value().double_value)
        max_steer_angle = (
            self._node.get_parameter("vehicle.max_steer_angle").get_parameter_value().double_value)
        max_acceleration = (
            self._node.get_parameter("vehicle.max_acceleration").get_parameter_value().double_value
    )

        # set base and derived params from vehicle base params (see AW.Universe: vehicle_info.cpp)
        self._vehicle_length = front_overhang + wheel_base + rear_overhang
        self._vehicle_width = left_overhang + wheel_tread + right_overhang
        self._vehicle_wheelbase = wheel_base
        self._vehicle_wb_front_axle = wb_front_axle
        self._vehicle_wb_rear_axle = wb_rear_axle
        self._vehicle_max_steer_angle = max_steer_angle
        self._vehicle_max_acceleration = max_acceleration

    def create_ego_with_cur_location(self):
        """Create a new ego vehicle with current position for visualization."""
        ego_vehicle_id = self._node.scenario_handler.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        if self.last_trajectory is None:
            return DynamicObstacle(
                ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state
            )
        else:
            pred_traj = TrajectoryPrediction(
                self._node._awtrajectory_to_crtrajectory(
                    1, self.ego_vehicle_state.time_step, self._node.last_trajectory.points
                ),
                ego_vehicle_shape,
            )
            return DynamicObstacle(
                ego_vehicle_id,
                ego_vehicle_type,
                ego_vehicle_shape,
                self.ego_vehicle_state,
                prediction=pred_traj,
            )
