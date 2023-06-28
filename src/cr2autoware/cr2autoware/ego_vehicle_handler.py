import os
import typing
from typing import Any
from typing import Dict
from typing import Optional

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.state import CustomState
from rcl_interfaces.msg import ParameterValue
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher

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
    _node: "Cr2Auto"
    _last_msg: Dict[str, Any] = {}

    # Publishers
    _OBSTACLE_PUBLISHER: Publisher

    @property
    def ego_vehicle(self):
        return self._ego_vehicle

    @ego_vehicle.setter
    def ego_vehicle(self, ego_vehicle) -> None:
        self._ego_vehicle = ego_vehicle

    @property
    def ego_vehicle_state(self) -> Optional[CustomState]:
        return self._ego_vehicle_state

    @ego_vehicle_state.setter
    def ego_vehicle_state(self, ego_vehicle_state: Optional[CustomState]) -> None:
        self._ego_vehicle_state = ego_vehicle_state

    def __init__(self, node: "Cr2Auto"):
        self._node = node
        self._logger = node.get_logger().get_child("ego_vehicle_handler")

        # Get parameters from the node
        self._init_parameters()

    def _init_parameters(self) -> None:
        def _get_parameter(name: str) -> ParameterValue:
            return self._node.get_parameter(name).get_parameter_value()

        self.MAP_PATH = _get_parameter("map_path").string_value
        if not os.path.exists(self.MAP_PATH):
            raise ValueError("Can't find given map path: %s" % self.MAP_PATH)

        self.VERBOSE = _get_parameter("detailed_log").bool_value
        if self.VERBOSE:
            from rclpy.logging import LoggingSeverity

            self._logger.set_level(LoggingSeverity.DEBUG)

    def update_ego_vehicle(self):
        """Update the commonroad scenario with the latest vehicle state and obstacle messages received."""
        # process last state message
        if self.current_vehicle_state is not None:
            self._process_current_state()
        else:
            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("has not received a vehicle state yet!")
            return

    def create_ego_vehicle_info(self):
        """Compute the dimensions of the ego vehicle."""
        cg_to_front = (
            self._node.get_parameter("vehicle.cg_to_front").get_parameter_value().double_value
        )
        cg_to_rear = (
            self._node.get_parameter("vehicle.cg_to_rear").get_parameter_value().double_value
        )
        width = self._node.get_parameter("vehicle.width").get_parameter_value().double_value
        front_overhang = (
            self._node.get_parameter("vehicle.front_overhang").get_parameter_value().double_value
        )
        rear_overhang = (
            self._node.get_parameter("vehicle.rear_overhang").get_parameter_value().double_value
        )
        self.vehicle_length = front_overhang + cg_to_front + cg_to_rear + rear_overhang
        self.vehicle_width = width
        self.vehicle_wheelbase = cg_to_front + cg_to_rear

    def _create_ego_with_cur_location(self):
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
