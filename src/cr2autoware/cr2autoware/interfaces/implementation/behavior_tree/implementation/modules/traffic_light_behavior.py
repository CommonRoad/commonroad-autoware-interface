import py_trees
from ...base.base_tree import BaseTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightState
from cr2autoware.common.configuration import BehaviorPlannerParams
from cr2autoware.common.configuration import CR2AutowareParams
from cr2autoware.interfaces.implementation.behavior_tree.behavior_utils import calculate_current_position_index
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from typing import List, Set, Dict
from ...behavior_utils import copy_from_blackboard, BehaviorScenarioParams
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import Point, Polygon
from abc import abstractmethod
from visualization_msgs.msg import MarkerArray, Marker
from cr2autoware.common.utils.transform import utm2map
from geometry_msgs.msg import Point as PointMsg
from shapely.geometry import LineString
import matplotlib.pyplot as plt
import json


class TrafficLightsTree(BaseTree):
    """
    Submodule for traffic light handling.

    :var logger: ROS2 node logger
    :var verbose: Flag for verbose logging
    :var root: Root node of the behavior tree
    """
    def __init__(self, logger: RcutilsLogger, verbose: bool):
        super(TrafficLightsTree, self).__init__(logger, verbose)
        self.root = self.create_behavior_tree()

    def create_behavior_tree(self):
        root = Sequence(name="TrafficLightModule", memory=False)
        
        # Initialize Sub Module
        update_module = TrafficLightUpdateAction(name="TrafficLightUpdate", logger=self.logger)

        # Check for traffic lights
        check_for_traffic_lights = Selector(name="CheckForTrafficLights", memory=False)

        # Check distance to traffic lights
        traffic_light_out_of_range = TrafficLightOutOfRangeCondition(name="TrafficLightOutOfRangeCondition", logger=self.logger)

        # Error Handling
        error_handling = ErrorHandlingAction(name="ErrorHandlingAction", logger=self.logger)

        # Traffic light handling
        traffic_light_handling = Sequence(name="TrafficLightHandling", memory=False)

        handle_traffic_light_cycle = Selector(name="HandleTrafficLightCycle", memory=False)

        yellow_light = Sequence(name="YellowLight", memory=False)
        red_light = Sequence(name="RedLight", memory=False)
        green_light = Sequence(name="GreenLight", memory=False)

        yellow_condition = YellowLightCondition(name="Yellow", logger=self.logger)
        red_condition = RedLightCondition(name="Red", logger=self.logger)
        green_condition = GreenLightCondition(name="Green", logger=self.logger)

        stop_position_calculation = StopPositionCalculationAction(name="StopPositionCalculationAction", logger=self.logger)
        yellow_light_decision = YellowLightDecisionAction(name="YellowLightDecision", logger=self.logger)
        yellow_light_handling = Selector(name="YellowLightHandling", memory=False)

        comfort_stop_yellow = Sequence(name="ComfortStopYellow", memory=False)
        no_stop = Sequence(name="NoStop", memory=False)

        brake_at_yellow_light = BrakeAtYellowLightCondition(name="BrakeAtYellowLight", logger=self.logger)
        comfort_braking_yellow = ComfortBrakingAction(name="ComfortBrakingYellow", logger=self.logger)
        comfort_braking_red = ComfortBrakingAction(name="ComfortBrakingRed", logger=self.logger)

        continue_driving_at_yellow_light = ContinueDrivingAtYellowLightCondition(name="ContinueDrivingAtYellowLight", logger=self.logger)
        continue_driving_yellow = ContinueDrivingAction(name="ContinueDrivingYellow", logger=self.logger)
        continue_driving_green = ContinueDrivingAction(name="ContinueDrivingGreen", logger=self.logger)

        publish_rviz_marker_green = PublishRVIZMarker(name="PublishRVIZMarkerGreen", logger=self.logger)
        publish_rviz_marker_yellow = PublishRVIZMarker(name="PublishRVIZMarkerYellow", logger=self.logger)
        publish_rviz_marker_red = PublishRVIZMarker(name="PublishRVIZMarkerRed", logger=self.logger)

        # Add children to the tree
        comfort_stop_yellow.add_children([brake_at_yellow_light, comfort_braking_yellow])
        no_stop.add_children([continue_driving_at_yellow_light, continue_driving_yellow])

        yellow_light_handling.add_children([comfort_stop_yellow, no_stop])

        yellow_light.add_children([yellow_condition, yellow_light_decision, yellow_light_handling, publish_rviz_marker_yellow])
        red_light.add_children([red_condition, comfort_braking_red, publish_rviz_marker_red])
        green_light.add_children([green_condition, continue_driving_green, publish_rviz_marker_green])

        handle_traffic_light_cycle.add_children([yellow_light, red_light, green_light])

        traffic_light_handling.add_children([stop_position_calculation, handle_traffic_light_cycle])

        check_for_traffic_lights.add_children([traffic_light_out_of_range, traffic_light_handling, error_handling])

        root.add_children([update_module, check_for_traffic_lights])

        return root

class TrafficLightBehavior(Behaviour):
    """
    Base class for traffic light behavior.

    :var name: Name of the behavior node
    :var logger: ROS2 node logger
    :var blackboard: Blackboard for the behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name)
        self._logger = logger

        self.init_blackboard(name)

    def init_blackboard(self, name) -> None:
        """
        Initialize the blackboard parameters.

        :param name: Name of the behavior node
        """
        # Global Blackboard
        self.blackboard = py_trees.blackboard.Client(name=(name + "Blackboard"))
        self.blackboard.register_key("global_params", access=py_trees.common.Access.READ)
        self.blackboard.register_key("params", access=py_trees.common.Access.READ)

        # Register keys for Global Inputs
        self.global_inputs = py_trees.blackboard.Client(name=(name + "GlobalInputs"), namespace="inputs")
        self.global_inputs.register_key("scenario", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_state", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("input_path", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("coordinate_system", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("input_path_curvilinear", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("origin_transformation", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("last_velocity_profile", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_position_curvilinear", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("z_coordinate", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_time_msg", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_position_index", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("empty_velocity_profile", access=py_trees.common.Access.READ)

        # Register keys for Module Inputs
        self.inputs = py_trees.blackboard.Client(name=(name + "Inputs"), namespace="/modules/traffic_lights/inputs")
        self.inputs.register_key("current_traffic_light_id", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("relevant_traffic_lights", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("relevant_lanelets", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("target_stop_position", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("traffic_light_lanelet_mapping", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("traffic_lights_in_range", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("decision_point", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("brake_at_yellow_light", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("velocity_profile_without_traffic_lights", access=py_trees.common.Access.READ)
        self.inputs.register_key("force_stop", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("force_pass", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("stop_line_position_curvilinear", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("distance_stop_line_vehicle_origin", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("target_stop_position_index", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("yellow_light_id", access=py_trees.common.Access.WRITE)

        # Register keys for Module Outputs
        self.outputs = py_trees.blackboard.Client(name=(name + "Outputs"), namespace="/modules/traffic_lights/outputs")
        self.outputs.register_key("velocity_profile", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("traffic_light_marker_array", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("scenario_params", access=py_trees.common.Access.WRITE)

        # Init Parameter
        self.global_params: CR2AutowareParams = self.blackboard.global_params
        self.params: BehaviorPlannerParams = self.blackboard.params

    def setup(self) -> None:
        pass

    def initialise(self) -> None:
        pass

    @abstractmethod
    def update(self) -> Status:
        pass

    def terminate(self, new_status) -> None:
        pass


class TrafficLightUpdateAction(TrafficLightBehavior):
    """
    Action to update the relevant traffic lights and lanelets.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)
        self.inputs.traffic_light_lanelet_mapping = {}

        # Calculate the distance between the stop line and the vehicle origin
        front_bumper_to_vehicle_origin = self.global_params.vehicle.front_overhang + self.global_params.vehicle.wheel_base
        distance_stop_line_vehicle_origin = self.params.distance_stop_line_to_vehicle_front_bumper + front_bumper_to_vehicle_origin
        self.inputs.distance_stop_line_vehicle_origin = distance_stop_line_vehicle_origin

    def setup(self):
        pass
    

    def initialise(self):
        pass

    def update(self):

        scenario: Scenario = self.global_inputs.get("scenario")

        # TrafficLight Position gives wrong position, so we take the position of the lanelet the traffic light is assigned to
        # Get all lanelets that are on the path and save them in the blackboard.
        # Get the start index of the relevant input path, also consider previous points, depending on the stop line overrun tolerance
        consider_previous_points = int(np.ceil(self.params.stop_line_overrun_tolerance))
        start_index = self.global_inputs.current_position_index - consider_previous_points
        if start_index < 0:
            start_index = 0
        relevant_input_path: List[List[float]] = self.global_inputs.input_path[start_index:]
        path: List[np.ndarray] = [np.array(p) for p in relevant_input_path]
        relevant_lanelets_nested_list: List[List[int]] = scenario.lanelet_network.find_lanelet_by_position(path)
        # Transform the lanelet ids from the nested list to a set
        relevant_lanelets: Set[int] = set()
        for lanelet_id_list in relevant_lanelets_nested_list:
            for lanelet_id in lanelet_id_list:
                relevant_lanelets.add(lanelet_id)
        self.inputs.relevant_lanelets= relevant_lanelets

        coordinate_system: CoordinateSystem = self.global_inputs.coordinate_system


        # Due to a limited projection domain, the convertion of the stop line position to curvilinear coordinates can lead to an error
        # Therefore, we check if the stop line position is in the projection domain
        def point_in_projection_domain(stop_line_position: np.ndarray, coordinate_system: CoordinateSystem) -> bool:
            projection_domain = coordinate_system.ccosy.projection_domain()
            polygon = Polygon(projection_domain)
            point = Point(stop_line_position[0], stop_line_position[1])
            return polygon.contains(point)

        # Get the relevant traffic lights from the scenario
        # Get all traffic lights that are on the path and save them in the blackboard.

        relevant_traffic_lights: Dict[int, np.ndarray] = {} # key: traffic light id, value: stop_line_position
        if self.params.no_stop_line_in_map:
            # If no stop line is defined in the scenario, we take the first vertex of the lanelet as stop line

            for lanelet_id in relevant_lanelets:
                lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)

                # Calculate the nearest stop line position to the vehicle
                stop_line_position_0 = lanelet.center_vertices[0]
                stop_line_position_end = lanelet.center_vertices[-1]

                # Transfrom the stop line position in curviliniear coordinates
                if not point_in_projection_domain(stop_line_position_0, coordinate_system) and not point_in_projection_domain(stop_line_position_end, coordinate_system):
                    self._logger.warning("[SVEN]Lanlet id: " + str(lanelet_id) + " is not in the projection domain!")
                    self._logger.warning("[SVEN]Stop line position is not in the projection domain! Stop line position: " + str(stop_line_position_0) + " and " + str(stop_line_position_end))
                    continue
                elif not point_in_projection_domain(stop_line_position_0, coordinate_system):
                    stop_line_position_curvilinear = coordinate_system.convert_to_curvilinear_coords(stop_line_position_end[0], stop_line_position_end[1])
                elif not point_in_projection_domain(stop_line_position_end, coordinate_system):
                    stop_line_position_curvilinear = coordinate_system.convert_to_curvilinear_coords(stop_line_position_0[0], stop_line_position_0[1])
                else:
                    stop_line_position_0_curv = coordinate_system.convert_to_curvilinear_coords(stop_line_position_0[0], stop_line_position_0[1])
                    stop_line_position_end_curv = coordinate_system.convert_to_curvilinear_coords(stop_line_position_end[0], stop_line_position_end[1])
                    # Check which stop line is closer to the vehicle
                    if stop_line_position_0_curv[0] < stop_line_position_end_curv[0]:
                        stop_line_position_curvilinear = stop_line_position_0_curv
                    else:
                        stop_line_position_curvilinear = stop_line_position_end_curv
                
                # skip if one of the stop line positions is not in the projection domain -> wrong lanelet
                    if np.abs(stop_line_position_curvilinear[1]) > 1.0:
                        continue

                for traffic_light_id in lanelet.traffic_lights:
                    # Check if the traffic light is already assigned to another lanelet
                    if traffic_light_id in relevant_traffic_lights:
                        self._logger.warning("[SVEN]Traffic light is already assigned to another lanelet! Traffic light id: " + str(traffic_light_id)
                                        + ", lanelet id: " + str(lanelet_id))
                        # Check which stop line is closer to the vehicle
                        relevant_stop_line_position_curvilinear = relevant_traffic_lights[traffic_light_id]
                        # Check which stop line is closer to the vehicle
                        if relevant_stop_line_position_curvilinear[0] < stop_line_position_curvilinear[0]:
                            # Continue with the current stop line, if the current stop line is closer to the vehicle
                            continue
                    
                    relevant_traffic_lights[traffic_light_id] = stop_line_position_curvilinear
        
        else:
            # If a stop line is defined in the scenario, we take the stop line as stop line
            # Calculate the nearest stop line position to the vehicle

            for lanelet_id in relevant_lanelets:
                lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)

                # Calculate the nearest stop line position to the vehicle
                if lanelet.stop_line is None:
                    self._logger.debug("[SVEN]Lanelet id: " + str(lanelet_id) + " has no stop line!")
                    continue
                else:
                    # check if stop line crosses the reference path
                    stop_line = LineString([(lanelet.stop_line.start[0], lanelet.stop_line.start[1]), (lanelet.stop_line.end[0], lanelet.stop_line.end[1])])

                    reference_path = LineString([(point[0], point[1]) for point in relevant_input_path])

                    if stop_line.intersects(reference_path):
                        # Get the intersection point
                        stop_line_point = stop_line.intersection(reference_path)
                        stop_line_position = np.array([stop_line_point.x, stop_line_point.y])
                        if not point_in_projection_domain(stop_line_position, coordinate_system):
                            self._logger.warning("[SVEN]Stop line position is not in the projection domain! Stop line position: " + str(stop_line_position))
                            continue
                        else:
                            stop_line_position_curvilinear = coordinate_system.convert_to_curvilinear_coords(stop_line_position[0], stop_line_position[1])
                            self._logger.debug("[SVEN]Stop line position: " + str(stop_line_position_curvilinear))
                            for traffic_light in lanelet.stop_line.traffic_light_ref:
                                relevant_traffic_lights[traffic_light] = stop_line_position_curvilinear
                    else:
                        self._logger.debug("[SVEN]Stop line does not intersect the reference path! Lanelet id: " + str(lanelet_id))
                        continue

        self.inputs.relevant_traffic_lights = relevant_traffic_lights
        self._logger.debug("[SVEN]Relevant Traffic Lights: " + str(relevant_traffic_lights))

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating TrafficLightUpdate to " + str(new_status))

class TrafficLightOutOfRangeCondition(TrafficLightBehavior):
    """
    Condition Node. Checks if there are traffic lights in range.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

        self.blackboard.register_key("/modules/lane_keeping/inputs/blackboard_condition", access=py_trees.common.Access.WRITE)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        current_position = self.global_inputs.current_position_curvilinear
        traffic_light_perception_range: float = self.params.traffic_light_perception_range
        relevant_traffic_lights: Dict[int, np.ndarray] = self.inputs.relevant_traffic_lights
        # Get all traffic lights that are in range and save them in the blackboard.
        traffic_lights_in_range: Dict[int, np.ndarray] = {} # key: traffic light id, value: stop_line_position


        # TODO: For now, we assume that there is max one traffic light in range
        # Get nearest traffic light index:
        min_distance_id = None
        min_distance = None        
        for traffic_light_id, stop_line_position in relevant_traffic_lights.items():

            distance = stop_line_position[0] - (self.inputs.distance_stop_line_vehicle_origin - self.params.distance_stop_line_to_vehicle_front_bumper) - current_position[0]
            # Check if traffic light has been passed
            if distance < 0.0:
                self._logger.debug("[SVEN]Stop line has been passed! Check if overrun tolerance is exceeded. Traffic light id: " + str(traffic_light_id))

                if np.abs(distance) > self.params.stop_line_overrun_tolerance:
                    self._logger.debug("[SVEN]Traffic light overrun tolerance exceeded! Traffic light id: " + str(traffic_light_id))
                    continue

            # Check if traffic light is in range
            if distance < traffic_light_perception_range:
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    min_distance_id = traffic_light_id
                elif distance == min_distance:
                    self._logger.warning("[SVEN]Multiple traffic lights with the same distance to the vehicle! Traffic light id: " + str(traffic_light_id))
                    self._logger.warning("[SVEN]StopLine has more than one traffic light assigned!")
                traffic_lights_in_range[traffic_light_id] = stop_line_position

        # Check if there is a traffic light in range
        if len(traffic_lights_in_range) > 0:
            # TODO: For now, we assume that there is max one traffic light in range
            # traffic light in range, save it in the blackboard and return FAILURE
            self.inputs.traffic_lights_in_range = {min_distance_id: traffic_lights_in_range[min_distance_id]}
            self.inputs.current_traffic_light_id = min_distance_id

            # Apply Lane Keeping
            self.blackboard.modules.lane_keeping.inputs.blackboard_condition = True

            # Apply reduced scenario box
            self.outputs.scenario_params = BehaviorScenarioParams(
                cr_obstacle_box_front=None,
                cr_obstacle_box_rear=3.0,
                cr_obstacle_box_side=None,
                cr_obstacle_box_prediction=False,
            )

            return Status.FAILURE
        else:
            # no traffic light in range, output the empty velocity profile, return SUCCESS
            self.outputs.velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)
            
            # Publish empty marker array
            marker_array = MarkerArray()
            del_marker = Marker()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)
            self.outputs.traffic_light_marker_array = marker_array

            # reset the scenario params
            self.outputs.scenario_params = BehaviorScenarioParams()

            return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating TrafficLightOutOfRangeCondition to " + str(new_status))        


class YellowLightCondition(TrafficLightBehavior):
    """
    Condition Node. Checks if current traffic light is active and yellow.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        scenario: Scenario = self.global_inputs.get("scenario")
        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        # Check if traffic light is yellow
        if traffic_light.active and traffic_light.color == TrafficLightState.YELLOW:
            return Status.SUCCESS
        else:
            # If the traffic light is not yellow, or switches from yellow to another color, reset the safe stop flag and the yellow light id
            self.inputs.force_stop = False
            self.inputs.force_pass = False
            self.inputs.yellow_light_id = None
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating YellowLightCondition to " + str(new_status))


class RedLightCondition(TrafficLightBehavior):
    """
    Condition Node. Checks if current traffic light is active and red.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        scenario: Scenario = self.global_inputs.get("scenario")
        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        # Check if traffic light is red or red-yellow
        if traffic_light.active and (traffic_light.color == TrafficLightState.RED or traffic_light.color == TrafficLightState.RED_YELLOW):
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating RedLightCondition to " + str(new_status))


class GreenLightCondition(TrafficLightBehavior):
    """
    Condition Node. Checks if current traffic light is active and green.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        scenario: Scenario = self.global_inputs.get("scenario")

        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        # Check if traffic light is green
        if traffic_light.active and traffic_light.color == TrafficLightState.GREEN:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating GreenLightCondition to " + str(new_status))

class StopPositionCalculationAction(TrafficLightBehavior):
    """
    Action Node. Calculates the decision point for the vehicle in front of the traffic light.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)
        self.inputs.force_stop = False

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        traffic_lights_in_range: Dict[int, np.ndarray] = self.inputs.traffic_lights_in_range
        traffic_light_id = self.inputs.current_traffic_light_id
        # Get the stop line position of the traffic light
        stop_line_position_curvilinear = traffic_lights_in_range[traffic_light_id]

        # Save the hold position in the blackboard
        self.inputs.stop_line_position_curvilinear = stop_line_position_curvilinear
        self.inputs.target_stop_position = stop_line_position_curvilinear[0] - self.inputs.distance_stop_line_vehicle_origin
        self.inputs.target_stop_position_index = calculate_current_position_index(np.array([self.inputs.target_stop_position]), self.global_inputs.input_path_curvilinear)

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating StopPositionCalculationAction to " + str(new_status))

class YellowLightDecisionAction(TrafficLightBehavior):
    """
    Action Node. Calculates the decision point for the vehicle in front of the traffic light.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)
        self.inputs.force_stop = False
        self.inputs.force_pass = False
        self.inputs.yellow_light_id = None
        self.first_position_index = None
        self.min_pass_line_velo = None
        self.iteration_data = []
        self.average_velocity = None

    def setup(self):
        pass

    def initialise(self):
        pass

    def plot_decision_graph(self):
        """
        Plots a graph with distance on the x-axis and velocity on the y-axis, with the decision point.
        """
        plt.figure(figsize=(10, 6))
        
        # Define x and y axis ranges
        x_range = np.linspace(-self.params.stop_line_overrun_tolerance, self.params.traffic_light_perception_range, 100)
        y_range = np.linspace(0, 70 / 3.6, 100)  # 70 km/h in m/s

        # Plot the decision line
        comfort_braking_x = [self.comfort_braking_distance(velocity) for velocity in y_range]
        plt.plot(comfort_braking_x, y_range, color='#FF8C00', label='Comfort Braking Line')

        # Plot the pass yellow light line
        min_pass_line_y = [self.min_pass_line_velocity(distance) for distance in x_range]
        plt.plot(x_range, min_pass_line_y, color='#FFA500', label='Pass Yellow Light')

        # Plot the iteration data
        for i, (distance, velocity, average_velocity) in enumerate(self.iteration_data):
            plt.scatter(distance, velocity, label=f'Iteration {i+1}' if i == 0 else "")
            plt.text(distance, velocity, str(i+1))

            if average_velocity is not None:
                plt.scatter(distance, average_velocity, label=f'Average Velocity {i+1}' if i == 0 else "")
                plt.text(distance, average_velocity, str(i+1)+".avg")

        plt.xlabel('Distance to Stop Line (m)')
        plt.ylabel('Velocity (m/s)')
        plt.title('YellowLightDecision')
        plt.legend()
        plt.grid(True)
        
        parent_directory = '/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/cr2autoware/interfaces/implementation/behavior_tree/output/traffic_light_module'

        # Save the plot as an SVG file
        plt.savefig(parent_directory + '/yellow_light_decision.svg')
        plt.close()

        # Save datato json file
        with open(parent_directory + '/yellow_light_decision.json', 'w') as f:
            json.dump({
                "x_axis": x_range.tolist(),
                "y_axis": y_range.tolist(),
                "comfort_braking_x": comfort_braking_x,
                "comfort_braking_y": y_range.tolist(),
                "min_pass_line_x": x_range.tolist(),
                "min_pass_line_y": min_pass_line_y,
                "iterations": [i for i in range(1, len(self.iteration_data)+1)],
                "distance": [data[0] for data in self.iteration_data],
                "velocity": [data[1] for data in self.iteration_data],
                "average_velocity": [data[2] for data in self.iteration_data]
            }, f, indent=4)
    
    def comfort_braking_distance(self, current_velocity: float) -> float:
        """
        Calculate the braking distance for a given velocity with the maximum comfort deceleration. Also consider the system delay.

        :param current_velocity: Current velocity of the vehicle
        :return: Braking distance
        """
        braking_distance = self.params.yellow_light_rollout_distance + (current_velocity ** 2) / (2 * self.params.max_comfort_deceleration) + self.params.system_delay * current_velocity
        return max(braking_distance, 0.0)

    def min_pass_line_velocity(self, distance: float) -> float:
        """
        Calculate the minimum velocity to pass the yellow light line at a given distance.

        :param distance: Distance to the stop line
        :return: Minimum velocity to pass the yellow light line
        """
        return max(distance / (self.params.yellow_light_time - self.params.system_delay), 0.0)
    
    def check_for_min_pass_line_velocity(self, current_velocity: float) -> bool:
        """
        Check if the current velocity is greater than the minimum velocity to pass the yellow light line.

        :param current_velocity: Current velocity of the vehicle
        :return: True, if the vehicle can pass the yellow light line
        """

        return current_velocity > self.min_pass_line_velo
    
    def check_for_comfort_braking_distance(self, distance: float, current_velocity: float) -> bool:
        """
        Check if the vehicle can brake within the braking distance.

        :param distance: Distance to the stop line
        :param current_velocity: Current velocity of the vehicle
        :return: True, if the vehicle can brake within the braking distance
        """
        braking_distance = self.comfort_braking_distance(current_velocity)
        return distance >= braking_distance

    def update(self):

        # Get the stop line position of the traffic light
        stop_line_position = self.inputs.stop_line_position_curvilinear

        # Get the current position of the vehicle
        current_position_curvilinear = self.global_inputs.current_position_curvilinear
        current_velocity = self.global_inputs.current_state.velocity

        # Calculate the distance between the current position and the stop line position
        distance = (stop_line_position[0] - (self.inputs.distance_stop_line_vehicle_origin - self.params.distance_stop_line_to_vehicle_front_bumper) - current_position_curvilinear[0])

        # For the first iteration, calculate the minimum velocity to pass the yellow light line
        if self.inputs.yellow_light_id != self.inputs.current_traffic_light_id:
            self.inputs.yellow_light_id = self.inputs.current_traffic_light_id
            # Calculate the minimum velocity to pass the yellow light line
            self.min_pass_line_velo = self.min_pass_line_velocity(distance)
            self.first_position_index = self.global_inputs.current_position_index
        
        # for debug plotting
        min_velo_check = False

        if distance >= 0.0:
            # Vehicle is in front of the stop line
            # Check if the vehicle can break within the comfort braking distance
            if self.check_for_comfort_braking_distance(distance, current_velocity):
                # Vehicle can brake within the comfort braking distance
                # TODO: COMFORT BRAKING
                self._logger.debug("[SVEN]Vehicle can brake within the comfort braking distance!")
                self.inputs.force_stop = True
                brake_at_yellow_light = True
            elif self.inputs.force_stop:
                self._logger.debug("[SVEN]Force STOP!")
                brake_at_yellow_light = True
            else:
                # Vehicle can not brake within the comfort braking distance
                # Check if the vehicle can pass the yellow light line with the current velocity profile
                min_velo_check = True
                if self.check_for_min_pass_line_velocity(current_velocity):
                    # vehicle can pass the yellow light line with the current velocity profile
                    self._logger.debug("[SVEN]Vehicle can pass the yellow line in time! Min Velocity: " + str(self.min_pass_line_velo))
                    self.inputs.force_pass = True
                    brake_at_yellow_light = False
                else:
                    # check if force pass is performed
                    if self.inputs.force_pass:
                        self._logger.debug("[SVEN]Force PASS!")
                        brake_at_yellow_light = False
                    else:
                        # Vehicle can not pass the yellow light line with the current velocity profile
                        # TODO: EMERGENCY BRAKING
                        self._logger.debug("[SVEN]Vehicle can not pass the yellow line in time! Braking!")
                        brake_at_yellow_light = True
                        self.inputs.force_stop = True

        # Vehicle passed the stop line but is within the overrun tolerance (do not use all of the overrun tolerance, to avoid the traffic light being passed (out of range))
        elif distance + (0.5 * self.params.stop_line_overrun_tolerance) >= 0.0:
            # Consider the case that the vehicle is almost standing
            if current_velocity < 1.0:
                # Vehicle is almost standing
                # always stop
                self._logger.debug("[SVEN]Vehicle passed the stop line and is almost standing! Braking!")
                brake_at_yellow_light = True
                self.inputs.force_stop = True

            # Vehicle is moving
            else:
                # check if safe stop is performed
                if self.inputs.force_stop:
                    self._logger.debug("[SVEN]Force STOP!")
                    brake_at_yellow_light = True
                else:
                    # Continue driving
                    brake_at_yellow_light = False
                    self._logger.debug("[SVEN]Vehicle passed the stop line and is moving. Continue driving!")
        else:
            # Vehicle already passed the stop Line and overrun tolerance
            if self.inputs.force_stop:
                self._logger.debug("[SVEN]Force STOP!")
                brake_at_yellow_light = True
            else:
                brake_at_yellow_light = False
                self._logger.debug("[SVEN]Vehicle passed the stop line and is moving. Continue driving!")

        # Save the decision in the blackboard
        self.inputs.brake_at_yellow_light = brake_at_yellow_light

        # Store the current velocity and distance to the stop line
        if current_velocity > 0.1:
            if min_velo_check:
                self.iteration_data.append((distance, current_velocity, self.average_velocity))
            else:
                self.iteration_data.append((distance, current_velocity, None))

        # Plot the graph
        if self.params.plot_decision_graph:
            self.plot_decision_graph()

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating DecisionPointCalculationAction to " + str(new_status))


class BrakeAtYellowLightCondition(TrafficLightBehavior):
    """
    Condition Node. Checks if the decision point is ahead of the vehicle.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        # Check if the decision point is ahead of the vehicle
        brake_at_yellow_light = self.inputs.brake_at_yellow_light
        if brake_at_yellow_light:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating DecisionPointAheadCondition to " + str(new_status))


class ContinueDrivingAtYellowLightCondition(TrafficLightBehavior):
    """
    Condition Node. Checks if the decision point is behind the vehicle.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        brake_at_yellow_light = self.inputs.brake_at_yellow_light
        if not brake_at_yellow_light:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating DecisionPointBehindCondition to " + str(new_status))


class EmergencyBrakingAction(TrafficLightBehavior):
    """
    Action Node. Calculates the velocity profile for emergency braking.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        # TODO: WIP, this is comfort braking, not emergency braking

        # Get the index of the decision point in the input path
        stop_point_index = self.inputs.target_stop_position_index
        # Define the new velocity profile
        velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)

        # Set the velocity profile for the decision point and all points behind to zero
        velocity_profile[stop_point_index:] = 0.0

        # Calculate a rollout velocity profile for comfort braking
        if self.params.comfort_rollout:
            comfort_rollout_distance_int = int(np.ceil(self.params.comfort_rollout_distance))
            comfort_rollout_index = stop_point_index - comfort_rollout_distance_int
            if comfort_rollout_index < 0:
                comfort_rollout_index = 0
            velocity_profile[comfort_rollout_index:stop_point_index] = self.params.comfort_rollout_speed

        self.outputs.velocity_profile = velocity_profile

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating EmergencyBrakingAction to " + str(new_status))


class ContinueDrivingAction(TrafficLightBehavior):
    """
    Action Node. Sets the velocity profile for continuing driving.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)
   
    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        self.outputs.velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)
        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ContinueDrivingAction to " + str(new_status))


class ComfortBrakingAction(TrafficLightBehavior):
    """
    Action Node. Calculates the velocity profile for comfort braking.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        # Get the index of the decision point in the input path
        stop_point_index = self.inputs.target_stop_position_index
        # Define the new velocity profile
        velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)

        # Set the velocity profile for the decision point and all points behind to zero
        velocity_profile[stop_point_index:] = 0.0

        # Calculate a rollout velocity profile for comfort braking
        if self.params.comfort_rollout:
            comfort_rollout_distance_int = int(np.ceil(self.params.comfort_rollout_distance))
            comfort_rollout_index = stop_point_index - comfort_rollout_distance_int
            if comfort_rollout_index < 0:
                comfort_rollout_index = 0
            velocity_profile[comfort_rollout_index:stop_point_index] = self.params.comfort_rollout_speed

        self.outputs.velocity_profile = velocity_profile

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ComfortBrakingAction to " + str(new_status))
    

class PublishRVIZMarker(TrafficLightBehavior):
    """
    Visualization Node for RVIZ. Publishes the stop line and decision point as markers in RVIZ.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)
    
    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):

        if self.params.publish_traffic_light_markers:
            scenario: Scenario = self.global_inputs.get("scenario")
            traffic_light_id = self.inputs.current_traffic_light_id
            traffic_light = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
            coordinate_system: CoordinateSystem = self.global_inputs.get("coordinate_system")
            
            try:
                stop_line_curv = copy_from_blackboard(self.inputs.stop_line_position_curvilinear)
                stop_line_cartesian = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], 0.0)

                stop_line_cart_min = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], -1.5)
                stop_line_cart_max = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], 1.5)
            except:
                stop_line_cartesian = None
                stop_line_cart_min = None
                stop_line_cart_max = None
            
            try: 
                # TODO: Visualization of new YellowLightDecisionAction?
                decision_point_curv = copy_from_blackboard(self.inputs.decision_point)
                decision_point_cartesian = coordinate_system.convert_to_cartesian_coords(decision_point_curv, 0.0)

                decision_point_cart_min = coordinate_system.convert_to_cartesian_coords(decision_point_curv, -1.5)
                decision_point_cart_max = coordinate_system.convert_to_cartesian_coords(decision_point_curv, 1.5)
            except:
                decision_point_cartesian = None
                decision_point_cart_min = None
                decision_point_cart_max = None

            z = self.global_inputs.get("z_coordinate")
            
            positions_lines = [stop_line_cart_min, stop_line_cart_max, decision_point_cart_min, decision_point_cart_max]
            positions_text = [stop_line_cartesian, decision_point_cartesian]
            text = ["StopLine", "DecisionPoint"]
            positions_aw_lines = []
            positions_aw = []

            # convert positions to AW coordinate system
            for pos in positions_lines:
                if pos is None:
                    continue
                positions_aw_lines.append(utm2map(self.global_inputs.get("origin_transformation"), pos))

            for pos in positions_text:
                if pos is None:
                    continue
                positions_aw.append(utm2map(self.global_inputs.get("origin_transformation"), pos))

            marker_array = MarkerArray()
            del_marker = Marker()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.global_inputs.get("current_time_msg")
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.5
            marker.color.a = 1.0

            if traffic_light.color == TrafficLightState.RED or traffic_light.color == TrafficLightState.RED_YELLOW:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif traffic_light.color == TrafficLightState.YELLOW:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif traffic_light.color == TrafficLightState.GREEN:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.id = traffic_light_id
            marker.ns = "traffic_light"
            marker.points = []
            for pos in positions_aw_lines:
                marker.points.append(PointMsg(x=pos.x, y=pos.y, z=z))

            marker_array.markers.append(marker)

            for pos in positions_aw:
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.global_inputs.get("current_time_msg")
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = 0.5
                text_marker.color.a = 1.0
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.text = text[positions_aw.index(pos)] + " " + str(traffic_light_id)
                text_marker.id = -traffic_light_id
                text_marker.ns = text[positions_aw.index(pos)] + " " + str(traffic_light_id)
                text_marker.pose.position.x = pos.x
                text_marker.pose.position.y = pos.y + 0.3
                text_marker.pose.position.z = z
                marker_array.markers.append(text_marker)

        else:
            marker_array = MarkerArray()
            del_marker = Marker()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)

        self.outputs.traffic_light_marker_array = marker_array

        # TODO: WIP, reset blackboard values for next cycle
        self.inputs.stop_line_position_curvilinear = None
        self.inputs.target_stop_position = None
        self.inputs.decision_point = None
            
        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating PublishRVIZMarker to " + str(new_status))


class ErrorHandlingAction(TrafficLightBehavior):
    """
    Action Node. Handles errors that occur during the traffic light behavior.

    :var logger: ROS2 node logger
    :var blackboard: Blackboard for behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        # When Traffic Light is inactive, or other errors occur, the vehicle should continue driving (e.g. car stops behind the stop line)

        # When in Error State, velocity profile is not changed
        self.outputs.velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)

        # Publish blue marker for error handling
        traffic_light_id = self.inputs.current_traffic_light_id
        coordinate_system: CoordinateSystem = self.global_inputs.get("coordinate_system")
        
        try:
            stop_line_curv = copy_from_blackboard(self.inputs.stop_line_position_curvilinear)
            stop_line_cartesian = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], 0.0)

            stop_line_cart_min = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], -1.5)
            stop_line_cart_max = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], 1.5)
        except:
            stop_line_cartesian = None
            stop_line_cart_min = None
            stop_line_cart_max = None

        z = self.global_inputs.get("z_coordinate")
        
        positions_lines = [stop_line_cart_min, stop_line_cart_max]
        positions_text = [stop_line_cartesian]
        text = ["StopLine ErrorHandling"]
        positions_aw_lines = []
        positions_aw = []

        # convert positions to AW coordinate system
        for pos in positions_lines:
            if pos is None:
                continue
            positions_aw_lines.append(utm2map(self.global_inputs.get("origin_transformation"), pos))

        for pos in positions_text:
            if pos is None:
                continue
            positions_aw.append(utm2map(self.global_inputs.get("origin_transformation"), pos))

        marker_array = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.global_inputs.get("current_time_msg")
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.id = traffic_light_id
        marker.ns = "traffic_light"
        marker.points = []
        for pos in positions_aw_lines:
            marker.points.append(PointMsg(x=pos.x, y=pos.y, z=z))

        marker_array.markers.append(marker)

        for pos in positions_aw:
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.global_inputs.get("current_time_msg")
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.5
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = text[positions_aw.index(pos)] + " " + str(traffic_light_id)
            text_marker.id = -traffic_light_id
            text_marker.ns = text[positions_aw.index(pos)] + " " + str(traffic_light_id)
            text_marker.pose.position.x = pos.x
            text_marker.pose.position.y = pos.y + 0.3
            text_marker.pose.position.z = z
            marker_array.markers.append(text_marker)

        self.outputs.traffic_light_marker_array = marker_array
        return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ErrorHandlingAction to " + str(new_status))
