from dataclasses import dataclass
import typing
from typing import Any
from typing import List
from typing import Sequence
from typing import Tuple

from commonroad.geometry.shape import Polygon
from commonroad.geometry.shape import ShapeGroup
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import Occupancy
from commonroad.prediction.prediction import SetBasedPrediction
from commonroad.scenario.scenario import Scenario
import numpy as np
from rcl_interfaces.msg import ParameterValue
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher
import spot
from visualization_msgs.msg import MarkerArray

from cr2autoware.common.utils.message import create_goal_region_marker

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


@dataclass
class SpotObstacle:
    """
    Result for a single obstacle from SPOT.

    :var obstacle_id: ID of the obstacle.
    :var predictions: List of predictions for the obstacle.
    """

    obstacle_id: int
    predictions: List["SpotPrediction"]

    @classmethod
    def from_spot_interface(cls, spot_result: Any) -> "SpotObstacle":
        """
        Convert a spot result from the spot interface to a SpotResult object.

        :param spot_result: Result of SPOT from the spot interface.
        :return: SpotResult object.
        """
        obstacle_id = int(spot_result[0])
        predictions = [
            SpotPrediction.from_spot_interface(prediction) for prediction in spot_result[1]
        ]

        return cls(obstacle_id, predictions)

    def get_occupancy_list(self) -> List[Occupancy]:
        """
        Convert the predictions of SPOT to a list of occupancies.

        :return: List of occupancies for the obstacle.
        """
        occupancy_list: List[Occupancy] = []
        for time_step, prediction in enumerate(self.predictions):
            # ignoring type, since ShapeGroup requires `List[Shape]`
            # It should require `Sequence[Shape]`
            shape_group = ShapeGroup(prediction.occupied_polygons)  # type: ignore
            occupancy = Occupancy(time_step + 1, shape_group)
            occupancy_list.append(occupancy)
        return occupancy_list


@dataclass
class SpotPrediction:
    """
    Prediction of SPOT for one time step and obstacle.


    :var velocities: List containing velocity information about the lanes, which are part of the prediction.
    :var occupied_polygons: List containing polygons of the occupancy
    """

    velocities: List[Tuple[float, float]]
    occupied_polygons: List[Polygon]

    @classmethod
    def from_spot_interface(cls, spot_prediction: Any) -> "SpotPrediction":
        """
        Convert a spot prediction from the spot interface to a SpotPrediction object.

        :param spot_prediction: Prediction of SPOT from the spot interface.
        :return: SpotPrediction object.
        """
        velocities = spot_prediction[0]
        polygons = cls._convert_vertex_list_to_polygons(spot_prediction[1])
        return cls(velocities, polygons)

    @classmethod
    def _convert_vertex_list_to_polygons(cls, vertices: Sequence[Sequence[float]]) -> List[Polygon]:
        """
        Convert a list of vertices as returned by SPOT to a list of polygons.

        SPOT returns all polygons as one list of vertices that needs to be split.
        This function converts the list of vertices to a list of polygons by
        creating a separate polygon whenever a vertex is reached, which has the
        same x and y coordinates as a preceding vertex.

        **Example:**

        SPOT returns the following list of vertices::

            [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0), (2, 0), (2, 1), (0, 1), (2, 0)]

        This list is converted to the following list of polygons::

            [Polygon([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]), Polygon([(2, 0), (2, 1), (0, 1), (2, 0)])]

        :param vertices: List of vertices as returned by SPOT.
        :return: List of polygons.
        :raises ValueError: If the list of vertices is not closed correctly.
        """
        shapes: List[Polygon] = []
        first_idx = 0
        while first_idx < len(vertices):
            first_vertex = vertices[first_idx]
            for next_idx, next_vertex in enumerate(vertices[first_idx + 1 :], start=first_idx + 1):
                # check if polygon is closed by comparing x and y coords
                if first_vertex[0] != next_vertex[0] or first_vertex[1] != next_vertex[1]:
                    if next_idx == len(vertices) - 1:
                        # last vertex reached, but polygon is not closed
                        raise ValueError(
                            "List of vertices is not closed correctly. \n" f"Vertices: {vertices}"
                        )
                    continue

                # check if polygon has at least 3 vertices
                if next_idx - first_idx < 3:
                    # TODO: handle logging
                    print(
                        "Warning: one polygon skipped when copying predicted occupancies to CommonRoad"
                    )
                    # skip this first_vertex and continue with the next one
                    first_idx += 1
                    break
                else:
                    # polygon is closed correctly and has at least 3 vertices
                    shapes.append(
                        Polygon(np.array(vertices[first_idx : next_idx + 1], dtype=float))
                    )
                    first_idx = next_idx + 1
                    break
        return shapes


class SpotHandler:
    """
    Handles communication with autoware for CommonRoad SPOT.

    Receives an up to date CommonRoad Scenario object and publishes results of SPOT based on that.

    ----------------
    **Publishers:**

    * _OCCUPANCY_REGION_PUBLISHER:
        * Description: Publishes predicted occupancy regions of the obstacles.
        * Topic: `/cr2autoware_marker_array`
        * Message Type: `visualization_msgs.msg.MarkerArray`

    ----------------
    :var VERBOSE: Flag to enable detailed logging.
    :var PLANNING_HORIZON: Planning horizon for SPOT.
    :var _logger: Logger for the SpotHandler.
    :var _node: reference to CR2Auto ROS2 node
    """

    # Constants and parameters
    VERBOSE: bool = False
    _OCCUPANCY_REGION_PUBLISHER: Publisher

    # Instance variables
    _logger: RcutilsLogger
    _node: "Cr2Auto"

    def __init__(self, node: "Cr2Auto") -> None:
        """
        Constructor for SpotHandler.

        :param node: reference to CR2Auto ROS2 node
        """
        self._logger = node.get_logger().get_child("spot")
        self._node = node

        self._init_parameters()

        self._init_publishers(self._node)

    def _init_parameters(self) -> None:
        """Retrieve required ROS params from self._node."""
        def _get_parameter(name: str) -> ParameterValue:
            return self._node.get_parameter(name).get_parameter_value()

        self.VERBOSE = _get_parameter("general.detailed_log").bool_value
        if self.VERBOSE:
            from rclpy.logging import LoggingSeverity

            self._logger.set_level(LoggingSeverity.DEBUG)
            spot.setLoggingMode(1)

        self.PLANNING_HORIZON = _get_parameter(
            "trajectory_planner.planning_horizon"
        ).double_value

    def _init_publishers(self, node: "Cr2Auto") -> None:
        """
        Initialize required publishers for self._node.
        
        :param node: reference to CR2Auto ROS2 node
        """
        self._OCCUPANCY_REGION_PUBLISHER = node.create_publisher(
            MarkerArray, "/cr2autoware_marker_array", 0
        )

    def update(
        self,
        scenario: Scenario,
        origin_transformation: Sequence[float],
        planning_problem: PlanningProblem,
    ) -> None:
        """
        Rerun SPOT and publish results to Autoware.
        
        :param scenario: CommonRoad scenario to predict obstacles for.
        :param origin_transformation: Transformation of the origin.
        :param planning_problem: Planning problem for SPOT.
        """
        # Register scenario
        if not self._register_scenario(scenario, planning_problem):
            self._logger.warning("Failed to register scenario with SPOT.")
            return
        self._logger.info("Registered scenario with SPOT.")

        # Run SPOT
        # TODO: Add params for num of steps and threads
        num_steps = int(self.PLANNING_HORIZON / scenario.dt)
        obstacles = self._run_spot(scenario, num_steps=num_steps)

        self._add_obstacles_to_scenario(scenario, obstacles)
        self._visualize_obstacles(obstacles, origin_transformation)

        # Remove scenario from SPOT
        spot.removeScenario(1)

    def _register_scenario(
        self, 
        scenario: Scenario, 
        planning_problem: PlanningProblem
    ) -> bool:
        """
        Register scenario with SPOT.
        
        :param scenario: CommonRoad scenario to register with SPOT.
        :param planning_problem: Planning problem for SPOT.
        :return: True if registration was successful, False otherwise.
        """
        # TODO: Add field of view
        field_of_view = np.empty([0, 2], float)
        result = spot.registerScenario(
            1,
            scenario.lanelet_network,
            scenario.dynamic_obstacles,
            [planning_problem],
            field_of_view,
        )

        # Apparently SPOT returns `False` on success?!?
        return not result

    def _run_spot(
        self, 
        scenario: Scenario, 
        num_steps: int = 15, 
        num_threads: int = 4
    ) -> List[SpotObstacle]:
        """
        Run SPOT for the given scenario.

        :param scenario: CommonRoad scenario to predict obstacles for.
        :param num_steps: Number of steps to predict obstacles for, defaults to 15
        :param num_threads: Number of threads to use for SPOT prediction, defaults to 4
        :return: List of predicted obstacles.
        """
        # start_time = float(self._node.get_clock().now().nanoseconds / 1e9)
        start_time = 0.0
        dt = scenario.dt
        end_time = num_steps * dt + start_time

        predicted = spot.doOccupancyPrediction(1, start_time, dt, end_time, num_threads)
        obstacles = [SpotObstacle.from_spot_interface(obstacle) for obstacle in predicted]
        self._logger.info(f"SPOT finished predicting {len(predicted)} obstacles.")
        return obstacles

    def _add_obstacles_to_scenario(
        self, 
        scenario: Scenario, 
        obstacles: Sequence[SpotObstacle], 
        initial_time_step: int = 1
    ) -> None:
        """
        Add predicted obstacles as `SetBasedPrediction` objects to scenario.

        :param scenario: CommonRoad scenario to add obstacles to.
        :param obstacles: SPOT obstacles to add to scenario.
        :param initial_time_step: Initial time step of the prediction, defaults to 1
        """
        for i, obstacle in enumerate(obstacles):
            scenario.dynamic_obstacles[i].prediction = SetBasedPrediction(
                initial_time_step, obstacle.get_occupancy_list()
            )

    # TODO: Make visualization more appealing
    def _visualize_obstacles(
        self, 
        obstacles: Sequence[SpotObstacle], 
        origin_transformation: Sequence[float]
    ) -> None:
        """
        Visualize obstacle in RViz.

        For now, we will use a simple goal region marker. We will later define a more sophisticated region marker.

        :param obstacles: List of obstacles to visualize.
        :param origin_transformation: Transformation of the origin.
        """
        marker_array = MarkerArray()
        marker_id = 0xFFFF
        for obstacle in obstacles:
            for time_step, prediction in enumerate(obstacle.predictions):
                for occupancy in prediction.occupied_polygons:
                    marker = create_goal_region_marker(occupancy, origin_transformation)
                    marker.ns = "spot"
                    marker.id = marker_id
                    marker.color.r = 68 / 255
                    marker.color.g = 228 / 255
                    marker.color.b = 242 / 255
                    marker.color.a = 1 - (time_step / len(obstacle.predictions))
                    marker_array.markers.append(marker)  # type: ignore
                    marker_id += 1

        self._OCCUPANCY_REGION_PUBLISHER.publish(marker_array)
