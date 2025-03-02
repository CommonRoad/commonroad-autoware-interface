import py_trees
from ...base.base_tree import BaseTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from commonroad.scenario.scenario import Scenario, Lanelet
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightState
from cr2autoware.common.configuration import BehaviorPlannerParams
from cr2autoware.common.configuration import CR2AutowareParams
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from typing import List, Set, Dict
from ...behavior_utils import copy_from_blackboard
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import Point, Polygon
from abc import abstractmethod
from visualization_msgs.msg import MarkerArray, Marker
from cr2autoware.common.utils.transform import utm2map
from geometry_msgs.msg import Point as PointMsg


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
        decision_point_calculation_yellow = DecisionPointCalculationAction(name="DecisionPointCalculationActionYellow", logger=self.logger)
        yellow_light_handling = Selector(name="YellowLightHandling", memory=False)

        comfort_stop_yellow = Sequence(name="ComfortStopYellow", memory=False)
        no_stop = Sequence(name="NoStop", memory=False)

        decision_point_ahead_yellow = DecisionPointAheadCondition(name="DecisionPointAheadYellow", logger=self.logger)
        comfort_braking_yellow = ComfortBrakingAction(name="ComfortBrakingYellow", logger=self.logger)
        comfort_braking_red = ComfortBrakingAction(name="ComfortBrakingRed", logger=self.logger)

        decision_point_behind_yellow = DecisionPointBehindCondition(name="DecisionPointBehindYellow", logger=self.logger)
        continue_driving_yellow = ContinueDrivingAction(name="ContinueDrivingYellow", logger=self.logger)
        continue_driving_green = ContinueDrivingAction(name="ContinueDrivingGreen", logger=self.logger)

        publish_rviz_marker_green = PublishRVIZMarker(name="PublishRVIZMarkerGreen", logger=self.logger)
        publish_rviz_marker_yellow = PublishRVIZMarker(name="PublishRVIZMarkerYellow", logger=self.logger)
        publish_rviz_marker_red = PublishRVIZMarker(name="PublishRVIZMarkerRed", logger=self.logger)

        # Add children to the tree
        comfort_stop_yellow.add_children([decision_point_ahead_yellow, comfort_braking_yellow])
        no_stop.add_children([decision_point_behind_yellow, continue_driving_yellow])

        yellow_light_handling.add_children([comfort_stop_yellow, no_stop])

        yellow_light.add_children([yellow_condition, decision_point_calculation_yellow, yellow_light_handling, publish_rviz_marker_yellow])
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
        self.inputs.register_key("decision_point_ahead", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("velocity_profile_without_traffic_lights", access=py_trees.common.Access.READ)
        self.inputs.register_key("safe_stop", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("stop_line_position_curvilinear", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("distance_stop_line_vehicle_origin", access=py_trees.common.Access.WRITE)

        # Register keys for Module Outputs
        self.outputs = py_trees.blackboard.Client(name=(name + "Outputs"), namespace="/modules/traffic_lights/outputs")
        self.outputs.register_key("velocity_profile", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("traffic_light_marker_array", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("d_min", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("d_max", access=py_trees.common.Access.WRITE)

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


    def setup(self):
            pass
    

    def initialise(self):
        self._logger.debug("Initialising TrafficLightUpdate")

    def update(self):
        self._logger.debug("Updating TrafficLightUpdate")

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

        for lanelet_id in relevant_lanelets:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            #TODO: ADD STOP LINE HANDLING here

            
            # If no stop line is defined in the scenario, we take the first vertex of the lanelet as stop line
            # Calculate the nearest stop line position to the vehicle
            stop_line_position_0 = lanelet.center_vertices[0]
            stop_line_position_end = lanelet.center_vertices[-1]

            # Transfrom the stop line position in curviliniear coordinates
            if not point_in_projection_domain(stop_line_position_0, coordinate_system) and not point_in_projection_domain(stop_line_position_end, coordinate_system):
                self._logger.warning("Lanlet id: " + str(lanelet_id) + " is not in the projection domain!")
                self._logger.warning("Stop line position is not in the projection domain! Stop line position: " + str(stop_line_position_0) + " and " + str(stop_line_position_end))
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

            for traffic_light_id in lanelet.traffic_lights:
                # Check if the traffic light is already assigned to another lanelet
                if traffic_light_id in relevant_traffic_lights:
                    self._logger.warning("Traffic light is already assigned to another lanelet! Traffic light id: " + str(traffic_light_id)
                                     + ", lanelet id: " + str(lanelet_id))
                    # Check which stop line is closer to the vehicle
                    relevant_stop_line_position_curvilinear = relevant_traffic_lights[traffic_light_id]
                    # Check which stop line is closer to the vehicle
                    if relevant_stop_line_position_curvilinear[0] < stop_line_position_curvilinear[0]:
                        # Continue with the current stop line, if the current stop line is closer to the vehicle
                        continue

                relevant_traffic_lights[traffic_light_id] = stop_line_position_curvilinear

        self.inputs.relevant_traffic_lights = relevant_traffic_lights
        self._logger.debug("Relevant Traffic Lights: " + str(relevant_traffic_lights))

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating TrafficLightUpdate to " + str(new_status))

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

    def setup(self):
        self._logger.debug("Setting up TrafficLightOutOfRangeCondition")

    def initialise(self):
        self._logger.debug("Initialising TrafficLightOutOfRangeCondition")

    def update(self):
        self._logger.debug("Updating TrafficLightOutOfRangeCondition")

        current_position = self.global_inputs.current_position_curvilinear
        traffic_light_perception_range: float = self.params.traffic_light_perception_range
        relevant_traffic_lights: Dict[int, np.ndarray] = self.inputs.relevant_traffic_lights
        # Get all traffic lights that are in range and save them in the blackboard.
        traffic_lights_in_range: Dict[int, np.ndarray] = {} # key: traffic light id, value: stop_line_position

        for traffic_light_id, stop_line_position in relevant_traffic_lights.items():
            # TODO: For now, we assume that there is max one traffic light in range
            # Get nearest traffic light index:
            min_distance_id = None
            min_distance = None

            distance = current_position[0] - stop_line_position[0]
            if distance < traffic_light_perception_range:
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    min_distance_id = traffic_light_id
                traffic_lights_in_range[traffic_light_id] = stop_line_position

        # Check if there is a traffic light in range
        if len(traffic_lights_in_range) > 0:
            # TODO: For now, we assume that there is max one traffic light in range
            # traffic light in range, save it in the blackboard and return FAILURE
            self.inputs.traffic_lights_in_range = {min_distance_id: traffic_lights_in_range[min_distance_id]}
            self.inputs.current_traffic_light_id = min_distance_id

            ############################################################################################
            # NO OVERTAKE BEFORE TRAFFIC LIGHT
            ############################################################################################
            # TODO: Refactor no overtake before traffic light in own module/function
            # Traffic light in range, so lateral offset restriction required (overtake not allowed)
            relevant_lanelets = self.inputs.relevant_lanelets

            # Get minimal width of the lanelets
            min_width = None
            for lanelet_id in relevant_lanelets:
                lanelet = self.global_inputs.scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
                width = minimum_width_lanelet(lanelet)
                self._logger.debug("Lanelet id: " + str(lanelet_id) + ", width: " + str(width))

                if min_width is None or width < min_width:
                    min_width = width
            
            self._logger.debug("Min width: " + str(min_width))

            # Get the vehicle width
            vehicle_width = self.global_params.vehicle.wheel_tread + self.global_params.vehicle.right_overhang + self.global_params.vehicle.left_overhang
            # Get default lateral offset
            default_d_min = self.global_params.rp_interface.d_min
            default_d_max = self.global_params.rp_interface.d_max

            # Calculate the lateral offset restriction
            d_abs = (min_width - vehicle_width) / 2
            # Keep a minimal buffer for safe trajectory planning
            d_min_buffer = self.params.d_minimal_buffer
            if d_abs < d_min_buffer:
                d_abs = d_min_buffer
            if d_abs < np.abs(default_d_min):
                self.outputs.d_min = -d_abs
            else:
                self.outputs.d_min = None
            
            if d_abs < np.abs(default_d_max):
                self.outputs.d_max = d_abs
            else:
                self.outputs.d_max = None

            ################################################################################################
            # END NO OVERTAKE BEFORE TRAFFIC LIGHT
            ################################################################################################

            return Status.FAILURE
        else:
            # no traffic light in range, output the empty velocity profile, return SUCCESS
            self.outputs.velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)

            ############################################################################################
            # No traffic light in range, so no latteral offset restriction required (overtake allowed)
            self.outputs.d_min = None
            self.outputs.d_max = None
            ############################################################################################
            return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("Terminating TrafficLightOutOfRangeCondition to " + str(new_status))        


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
        self._logger.debug("Setting up YellowLightCondition")

    def initialise(self):
        self._logger.debug("Initialising YellowLightCondition")

    def update(self):
        self._logger.debug("Updating YellowLightCondition")

        scenario: Scenario = self.global_inputs.get("scenario")
        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        # Check if traffic light is yellow
        if traffic_light.active and traffic_light.color == TrafficLightState.YELLOW:
            return Status.SUCCESS
        else:
            # If the traffic light is not yellow, or switches from yellow to another color, reset the safe stop flag
            self.inputs.safe_stop = False
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating YellowLightCondition to " + str(new_status))


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
        self._logger.debug("Setting up RedLightCondition")

    def initialise(self):
        self._logger.debug("Initialising RedLightCondition")

    def update(self):
        self._logger.debug("Updating RedLightCondition")

        scenario: Scenario = self.global_inputs.get("scenario")
        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        # Check if traffic light is red or red-yellow
        if traffic_light.active and (traffic_light.color == TrafficLightState.RED or traffic_light.color == TrafficLightState.RED_YELLOW):
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status):
        self._logger.debug("Terminating RedLightCondition to " + str(new_status))


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
        self._logger.debug("Setting up GreenLightCondition")

    def initialise(self):
        self._logger.debug("Initialising GreenLightCondition")

    def update(self):
        self._logger.debug("Updating GreenLightCondition")
        scenario: Scenario = self.global_inputs.get("scenario")

        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light: TrafficLight = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        # Check if traffic light is green
        if traffic_light.active and traffic_light.color == TrafficLightState.GREEN:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating GreenLightCondition to " + str(new_status))

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
        self.inputs.safe_stop = False

    def setup(self):
        self._logger.debug("Setting up StopPositionCalculationAction")

    def initialise(self):
        self._logger.debug("Initialising StopPositionCalculationAction")

    def update(self):
        self._logger.debug("Updating StopPositionCalculationAction")

        traffic_lights_in_range: Dict[int, np.ndarray] = self.inputs.traffic_lights_in_range
        traffic_light_id = self.inputs.current_traffic_light_id
        # Get the stop line position of the traffic light
        stop_line_position_curvilinear = traffic_lights_in_range[traffic_light_id]

        # Calculate the distance between the current position and the stop line position
        # Also consider vehicle front bumper to vehicle origin and the additional parameter distance_stop_line_to_vehicle_front_bumper
        # vehicle origin is on the rear axle
        front_bumper_to_vehicle_origin = self.global_params.vehicle.front_overhang + self.global_params.vehicle.wheel_base
        distance_stop_line_vehicle_origin = self.params.distance_stop_line_to_vehicle_front_bumper + front_bumper_to_vehicle_origin
        self.inputs.distance_stop_line_vehicle_origin = distance_stop_line_vehicle_origin

        # Save the hold position in the blackboard
        self.inputs.stop_line_position_curvilinear = stop_line_position_curvilinear
        self.inputs.target_stop_position = stop_line_position_curvilinear[0] - distance_stop_line_vehicle_origin

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating StopPositionCalculationAction to " + str(new_status))

class DecisionPointCalculationAction(TrafficLightBehavior):
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
        self.inputs.safe_stop = False

    def setup(self):
        self._logger.debug("Setting up DecisionPointCalculationAction")

    def initialise(self):
        self._logger.debug("Initialising DecisionPointCalculationAction")

    def update(self):
        self._logger.debug("Updating DecisionPointCalculationAction")

        # Get the stop line position of the traffic light
        stop_line_position = self.inputs.stop_line_position_curvilinear

        # Get the current position of the vehicle
        current_position_curvilinear = self.global_inputs.current_position_curvilinear
        current_velocity = self.global_inputs.current_state.velocity

        # Calculate the distance between the current position and the stop line position
        distance_stop_line_vehicle_origin = self.inputs.distance_stop_line_vehicle_origin
        distance = (stop_line_position[0] - distance_stop_line_vehicle_origin - current_position_curvilinear[0])

        # Calculate the braking distance, also consider the system delay
        braking_distance = (current_velocity ** 2) / (2 * self.params.max_comfort_deceleration) + self.params.system_delay * current_velocity
        braking_distance = max(braking_distance, 0.0)

        # Calculate the decision point
        decision_point = current_position_curvilinear[0] + (distance - braking_distance)

        if distance >= braking_distance:
            # Vehicle has not reached the decision point yet
            # Breaking distance is smaller than the distance to the stop line
            decision_point_ahead = True

        # Vehicle passed the decision point, but did not reach the stop line yet
        # Consider the case that the vehicle is almost standing
        elif distance + self.params.stop_line_overrun_tolerance >= 0.0:
            # Vehicle is standing or almost standing:
            if current_velocity < 1.5:
                # Vehicle is almost standing
                if distance + self.params.stop_line_overrun_tolerance >= braking_distance:
                    # Vehicle can brake within the overrun tolerance
                    # For this case, the vehicle should stop
                    decision_point_ahead = True
                    self.inputs.safe_stop = True
                else:
                    # Vehicle can not brake within the overrun tolerance
                    # For this case, the vehicle should continue driving
                    decision_point_ahead = False

            # Vehicle is moving
            else:
                # Breaking distance is greater than the distance to the stop line
                # Vehicle should contine driving
                if not self.inputs.safe_stop:
                    decision_point_ahead = False
                else:
                    # Safe stop is performed, keep the decision point ahead
                    decision_point_ahead = True
        elif distance < 0.0:
            # Vehicle already passed the stop Line
            # check if the decision point is behind the stop line overrun tolerance
            distance_vehicle_origin_to_stop_line = current_position_curvilinear[0] - stop_line_position[0]
            if distance_vehicle_origin_to_stop_line > self.params.stop_line_overrun_tolerance:
                # Vehicle already passed the stop line and overrun tolerance
                # No decision_point calculation required
                return Status.FAILURE

            if self.inputs.safe_stop:
                # Safe stop is performed, keep the decision point ahead
                decision_point_ahead = True
            else:
                # Vehicle already passed the stop line and decision point
                decision_point_ahead = False

        # Save the hold position in the blackboard
        self.inputs.target_stop_position = stop_line_position[0] - distance_stop_line_vehicle_origin
        # Save the decision point in the blackboard
        self.inputs.decision_point = decision_point
        self.inputs.decision_point_ahead = decision_point_ahead

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating DecisionPointCalculationAction to " + str(new_status))


class DecisionPointAheadCondition(TrafficLightBehavior):
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
        self._logger.debug("Setting up DecisionPointAheadCondition")

    def initialise(self):
        self._logger.debug("Initialising DecisionPointAheadCondition")

    def update(self):
        self._logger.debug("Updating DecisionPointAheadCondition")

        # Check if the decision point is ahead of the vehicle
        decision_point_ahead = self.inputs.decision_point_ahead
        if decision_point_ahead:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating DecisionPointAheadCondition to " + str(new_status))


class DecisionPointBehindCondition(TrafficLightBehavior):
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
        self._logger.debug("Setting up DecisionPointBehindCondition")

    def initialise(self):
        self._logger.debug("Initialising DecisionPointBehindCondition")

    def update(self):
        self._logger.debug("Updating DecisionPointBehindCondition")

        decision_point_ahead = self.inputs.decision_point_ahead
        if not decision_point_ahead:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating DecisionPointBehindCondition to " + str(new_status))


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
        self._logger.debug("Setting up EmergencyBrakingAction")

    def initialise(self):
        self._logger.debug("Initialising EmergencyBrakingAction")

    def update(self):
        self._logger.debug("Updating EmergencyBrakingAction")

        # TODO: WIP, this is comfort braking, not emergency braking

        target_stop_position = self.inputs.target_stop_position
        input_path_curvilinear = self.global_inputs.input_path_curvilinear

        # Get the index of the decision point in the input path
        stop_point_index = np.argmin(np.abs(input_path_curvilinear - target_stop_position))
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
        self._logger.debug("Terminating EmergencyBrakingAction to " + str(new_status))


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
        self._logger.debug("Setting up ContinueDrivingAction")

    def initialise(self):
        self._logger.debug("Initialising ContinueDrivingAction")

    def update(self):
        self._logger.debug("Updating ContinueDrivingAction")
        self.outputs.velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)
        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating ContinueDrivingAction to " + str(new_status))


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
        self._logger.debug("Setting up ComfortBrakingAction")

    def initialise(self):
        self._logger.debug("Initialising ComfortBrakingAction")

    def update(self):
        self._logger.debug("Updating ComfortBrakingAction")

        target_stop_position = self.inputs.target_stop_position
        input_path_curvilinear = self.global_inputs.input_path_curvilinear

        # Get the index of the decision point in the input path
        stop_point_index = np.argmin(np.abs(input_path_curvilinear - target_stop_position))
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
        self._logger.debug("Terminating ComfortBrakingAction to " + str(new_status))
    

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
        self._logger.debug("Setting up PublishRVIZMarker")

    def initialise(self):
        self._logger.debug("Initialising PublishRVIZMarker")

    def update(self):
        self._logger.debug("Updating PublishRVIZMarker")

        scenario: Scenario = self.global_inputs.get("scenario")
        traffic_light_id = self.inputs.current_traffic_light_id
        traffic_light = scenario.lanelet_network.find_traffic_light_by_id(traffic_light_id)
        coordinate_system: CoordinateSystem = self.global_inputs.get("coordinate_system")
        
        try:
            stop_line_curv = copy_from_blackboard(self.inputs.stop_line_position_curvilinear)
            stop_line_cartesian = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], 0.0)
            self._logger.debug("Stop Line: " + str(stop_line_cartesian))

            stop_line_cart_min = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], -1.5)
            stop_line_cart_max = coordinate_system.convert_to_cartesian_coords(stop_line_curv[0], 1.5)
        except:
            stop_line_cartesian = None
            stop_line_cart_min = None
            stop_line_cart_max = None
        
        try: 
            decision_point_curv = copy_from_blackboard(self.inputs.decision_point)
            decision_point_cartesian = coordinate_system.convert_to_cartesian_coords(decision_point_curv, 0.0)
            self._logger.debug("Decision Point: " + str(decision_point_cartesian))

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
            marker.points.append(PointMsg(x=pos.x, y=pos.y, z=(z+2.5)))

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
            text_marker.pose.position.z = z + 2.5
            marker_array.markers.append(text_marker)
        
        self.outputs.traffic_light_marker_array = marker_array

        # TODO: WIP, reset blackboard values for next cycle
        self.inputs.stop_line_position_curvilinear = None
        self.inputs.target_stop_position = None
        self.inputs.decision_point = None
            
        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating PublishRVIZMarker to " + str(new_status))


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
        self._logger.debug("Setting up ErrorHandlingAction")

    def initialise(self):
        self._logger.debug("Initialising ErrorHandlingAction")

    def update(self):
        # When Traffic Light is inactive, or other errors occur, the vehicle should continue driving (e.g. car stops behind the stop line)

        # When in Error State, velocity profile is not changed
        self.outputs.velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)

        # Delete all markers
        marker_array = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)
        self.outputs.traffic_light_marker_array = marker_array
        return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("Terminating ErrorHandlingAction to " + str(new_status))


# Utils

def minimum_width_lanelet(lanelet: Lanelet) -> float:
    """
    Calculate the minimum width of the lanelet by finding the minimum distance between left and right vertices.

    :param lanelet: lanelet of a CommonRoad scenario
    :return: The minimum width of the lanelet.
    """
    left_vertices = lanelet.left_vertices
    right_vertices = lanelet.right_vertices
    widths = np.linalg.norm(left_vertices - right_vertices, axis=1)
    return np.min(widths)
