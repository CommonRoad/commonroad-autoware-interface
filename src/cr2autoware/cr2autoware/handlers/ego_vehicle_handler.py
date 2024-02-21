# standard imports
import typing
from typing import Optional
from dataclasses import dataclass

# third party imports
import numpy as np

# ROS message imports
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry

# ROS imports
import rclpy

# commonroad imports
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.state import InitialState, FloatExactOrInterval

# cr2autoware imports
from .base import BaseHandler
import cr2autoware.common.utils.utils as utils

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


@dataclass(eq=False)
class EgoVehicleState(InitialState):
    """
    CommonRoad state class or the ego vehicle
    Extends the default initial state class by attribute steering angle
    """
    steering_angle: FloatExactOrInterval = None


class EgoVehicleHandler(BaseHandler):
    """
    # Update of the CR Ego vehicle with current vehicle state from Autoware.

    # This class should contain all methods related to updating the CommonRoad Ego Vehicle object with the current ego state from Autoware
    # current ego vehicle is a class attribute (e.g., self.ego_vehicle)
    # contains subscription/callbacks to current ego vehicle state from AW
    # contains all associated methods necessary for transformations etc...
    # outward interface to retrieve current ego vehicle: e.g., EgoVehicleHandler.get_ego_vehicle().
    """

    _vehicle_length: Optional[float] = None
    _vehicle_width: Optional[float] = None
    _vehicle_wheelbase: Optional[float] = None
    _vehicle_wb_front_axle: Optional[float] = None
    _vehicle_wb_rear_axle: Optional[float] = None
    _vehicle_max_steer_angle: Optional[float] = None
    _vehicle_max_acceleration: Optional[float] = None

    def __init__(self, node: "Cr2Auto"):
        # init base class
        super().__init__(node=node,
                         logger=node.get_logger().get_child("ego_vehicle_handler"),
                         verbose=node.verbose)

        # initialize subscriptions
        self._init_subscriptions()

        # init parameters: create ego vehicle dimension infos
        self._init_parameters()

        self._ego_vehicle_state: Optional[EgoVehicleState] = None
        self._current_vehicle_state: Odometry = None
        self._current_vehicle_acc: AccelWithCovarianceStamped = None
        self.new_pose_received: bool = False

    def _init_parameters(self):
        """Set the dimensions and kinematic parameters of the ego vehicle."""
        # get parameters from node
        wheel_base = self._get_param("vehicle.wheel_base").double_value
        wheel_tread = self._get_param("vehicle.wheel_tread").double_value
        wb_front_axle = self._get_param("vehicle.wb_front_axle").double_value
        wb_rear_axle = self._get_param("vehicle.wb_rear_axle").double_value
        front_overhang = self._get_param("vehicle.front_overhang").double_value
        rear_overhang = self._get_param("vehicle.rear_overhang").double_value
        left_overhang = self._get_param("vehicle.left_overhang").double_value
        right_overhang = self._get_param("vehicle.right_overhang").double_value
        max_steer_angle = self._get_param("vehicle.max_steer_angle").double_value
        max_acceleration = self._get_param("vehicle.max_acceleration").double_value

        # set base and derived params from vehicle base params (see AW.Universe: vehicle_info.cpp)
        self._vehicle_length = front_overhang + wheel_base + rear_overhang
        self._vehicle_width = left_overhang + wheel_tread + right_overhang
        self._vehicle_wheelbase = wheel_base
        self._vehicle_wb_front_axle = wb_front_axle
        self._vehicle_wb_rear_axle = wb_rear_axle
        self._vehicle_max_steer_angle = max_steer_angle
        self._vehicle_max_acceleration = max_acceleration

    def _init_subscriptions(self):
        # subscribe current state from odometry
        self._node.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.current_state_callback,
            1,
            callback_group=self._node.callback_group)

        # subscribe current acceleration (separate topic, currently not in /localization/kinematic_state)
        self._node.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self.current_acc_callback,
            1,
            callback_group=self._node.callback_group)

    def _init_publishers(self):
        pass

    @property
    def ego_vehicle_state(self) -> Optional[EgoVehicleState]:
        """CommonRoad ego vehicle state. Coordinates in CR frame"""
        return self._ego_vehicle_state

    @property
    def current_vehicle_state(self):
        """Autoware ego vehicle state msg. Coordinates in AW map frame"""
        return self._current_vehicle_state

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

    def current_state_callback(self, msg: Odometry) -> None:
        """
        Callback to current kinematic state of the ego vehicle.
        :param msg: current kinematic state message
        """
        self._current_vehicle_state = msg
        self.new_pose_received = True

    def current_acc_callback(self, msg: AccelWithCovarianceStamped) -> None:
        """
        Callback to current acceleration of the ego vehicle.
        NOTE: acceleration is currently not part of kinematic state message, that's why separate callback is required
        :param msg: current acceleration message
        """
        # store acceleration message
        self._current_vehicle_acc = msg

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
                pose_transformed = self._node.transform_pose(temp_pose_stamped, "map")
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

            acceleration = self._current_vehicle_acc.accel.accel.linear.x

            self._ego_vehicle_state = EgoVehicleState(
                position=position,
                orientation=orientation,
                velocity=self._current_vehicle_state.twist.twist.linear.x,
                yaw_rate=self._current_vehicle_state.twist.twist.angular.z,
                acceleration=acceleration,
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
            if self._VERBOSE:
                self._logger.info("has not received a vehicle state yet!")
            return

    def get_cr_ego_vehicle(self):
        """Create a new ego vehicle with current position for visualization."""
        ego_vehicle_id = self._node.scenario_handler.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        return DynamicObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state)
