import py_trees
from ...base.base_tree import BaseTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightState
from cr2autoware.common.configuration import BehaviorPlannerParams
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from typing import List, Set, Dict
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import Point, Polygon
from abc import abstractmethod
from visualization_msgs.msg import MarkerArray, Marker
from cr2autoware.common.utils.transform import utm2map
from geometry_msgs.msg import Point as PointMsg


# TODO: Create for each Behavior Module own Behavior Base Class with own Blackboard Clients for better maintainability
class TrafficLightsTree(BaseTree):
    def __init__(self, logger: RcutilsLogger, verbose: bool, config=None):
        super(TrafficLightsTree, self).__init__(logger, verbose, config)
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
        traffic_light_handling = Selector(name="TrafficLightHandling", memory=False)

        yellow_light = Sequence(name="YellowLight", memory=False)
        red_light = Sequence(name="RedLight", memory=False)
        green_light = Sequence(name="GreenLight", memory=False)

        yellow_condition = YellowLightCondition(name="Yellow", logger=self.logger)
        red_condition = RedLightCondition(name="Red", logger=self.logger)
        green_condition = GreenLightCondition(name="Green", logger=self.logger)

        stop_point_calculation_yellow = StopPointCalculationAction(name="StopPointCalculationActionYellow", logger=self.logger)
        stop_point_calculation_red = StopPointCalculationAction(name="StopPointCalculationActionRed", logger=self.logger)
        yellow_light_handling = Selector(name="YellowLightHandling", memory=False)
        red_light_handling = Selector(name="RedLightHandling", memory=False)

        comfort_stop_yellow = Sequence(name="ComfortStopYellow", memory=False)
        comfort_stop_red = Sequence(name="ComfortStopRed", memory=False)
        emergency_stop = Sequence(name="EmergencyStop", memory=False)
        no_stop = Sequence(name="NoStop", memory=False)

        stop_point_ahead_yellow = StopPointAheadCondition(name="StopPointAheadYellow", logger=self.logger)
        stop_point_ahead_red = StopPointAheadCondition(name="StopPointAheadRed", logger=self.logger)
        comfort_braking_yellow = ComfortBrakingAction(name="ComfortBrakingYellow", logger=self.logger)
        comfort_braking_red = ComfortBrakingAction(name="ComfortBrakingRed", logger=self.logger)

        stop_point_behind_yellow = StopPointBehindCondition(name="StopPointBehindYellow", logger=self.logger)
        stop_point_behind_red = StopPointBehindCondition(name="StopPointBehindRed", logger=self.logger)
        emergency_braking = EmergencyBrakingAction(name="EmergencyBraking", logger=self.logger)
        continue_driving_yellow = ContinueDrivingAction(name="ContinueDrivingYellow", logger=self.logger)
        continue_driving_green = ContinueDrivingAction(name="ContinueDrivingGreen", logger=self.logger)

        publish_rviz_marker_green = PublishRVIZMarker(name="PublishRVIZMarkerGreen", logger=self.logger)
        publish_rviz_marker_yellow = PublishRVIZMarker(name="PublishRVIZMarkerYellow", logger=self.logger)
        publish_rviz_marker_red = PublishRVIZMarker(name="PublishRVIZMarkerRed", logger=self.logger)

        # Add children to the tree
        comfort_stop_yellow.add_children([stop_point_ahead_yellow, comfort_braking_yellow])
        comfort_stop_red.add_children([stop_point_ahead_red, comfort_braking_red])
        no_stop.add_children([stop_point_behind_yellow, continue_driving_yellow])
        emergency_stop.add_children([stop_point_behind_red, emergency_braking])

        yellow_light_handling.add_children([comfort_stop_yellow, no_stop])
        red_light_handling.add_children([comfort_stop_red, emergency_stop])

        yellow_light.add_children([yellow_condition, stop_point_calculation_yellow, yellow_light_handling, publish_rviz_marker_yellow])
        red_light.add_children([red_condition, stop_point_calculation_red, red_light_handling, publish_rviz_marker_red])
        green_light.add_children([green_condition, continue_driving_green, publish_rviz_marker_green])

        traffic_light_handling.add_children([yellow_light, red_light, green_light])

        check_for_traffic_lights.add_children([traffic_light_out_of_range, traffic_light_handling, error_handling])

        root.add_children([update_module, check_for_traffic_lights])

        return root

class TrafficLightBehavior(Behaviour):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name)
        self._logger = logger

        self.init_blackboard(name)

    def init_blackboard(self, name):
        # Global Blackboard
        self.blackboard = py_trees.blackboard.Client(name=(name + "Blackboard"))
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

        # Register keys for Module Inputs
        self.inputs = py_trees.blackboard.Client(name=(name + "Inputs"), namespace="/modules/traffic_lights/inputs")
        self.inputs.register_key("current_traffic_light_id", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("relevant_traffic_lights", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("relevant_lanelets", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("target_stop_position", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("traffic_light_lanelet_mapping", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("traffic_lights_in_range", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("stop_point", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("stop_point_ahead", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("velocity_profile_without_traffic_lights", access=py_trees.common.Access.READ)

        # Register keys for Module Outputs
        self.outputs = py_trees.blackboard.Client(name=(name + "Outputs"), namespace="/modules/traffic_lights/outputs")
        self.outputs.register_key("velocity_profile", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("traffic_light_marker_array", access=py_trees.common.Access.WRITE)

        # Init Parameter
        self.params: BehaviorPlannerParams = self.blackboard.params

    def setup(self):
        pass

    def initialise(self):
        pass

    @abstractmethod
    def update(self) -> Status:
        pass

    def terminate(self, new_status):
        pass


class TrafficLightUpdateAction(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)
        self.inputs.traffic_light_lanelet_mapping = {}


    def setup(self):
            #TODO: Setup Method is not called!!
            # self._logger.debug("Setting up TrafficLightUpdate")
            # # create traffic light lanelet mapping
            # self._logger.debug("Creating traffic light lanelet mapping")
            # scenario: Scenario = self.global_inputs.get("scenario")
            # traffic_light_lanelet_mapping: Dict[int, int] = {}  # key: traffic light id, value: lanelet id
            # for lanelet_id in scenario.lanelet_network.lanelets:
            #     lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            #     for traffic_light_id in lanelet.traffic_lights:
            #         # Check if traffic light is already in the mapping
            #         if traffic_light_id in traffic_light_lanelet_mapping:
            #             raise ValueError("Traffic light is already assigned to another lanelet! Traffic light id: " + str(traffic_light_id)
            #                              + ", lanelet id: " + str(lanelet_id) + ", already assigned lanelet id: " 
            #                              + str(traffic_light_lanelet_mapping[traffic_light_id])
            #                              )
                    
            #         traffic_light_lanelet_mapping[traffic_light_id] = lanelet_id
        
            # self.inputs.traffic_light_lanelet_mapping = traffic_light_lanelet_mapping
        
            # self._logger.debug("Traffic light lanelet mapping: " + str(traffic_light_lanelet_mapping))
            pass
    

    def initialise(self):
        self._logger.debug("Initialising TrafficLightUpdate")

    def update(self):
        self._logger.debug("Updating TrafficLightUpdate")

        scenario: Scenario = self.global_inputs.get("scenario")

        # TrafficLight Position gives wrong position, so we take the position of the lanelet the traffic light is assigned to
        # Get all lanelets that are on the path and save them in the blackboard.
        self.logger.debug("input_path: " + str(self.global_inputs.input_path_curvilinear))
        path: List[np.ndarray] = [np.array(p) for p in self.global_inputs.input_path]
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
            # In current scenario, no stop line is defined, so we take the first vertex of the lanelet as stop line
            # Calculate the nearest stop line position to the vehicle
            stop_line_position_0 = lanelet.center_vertices[0]
            stop_line_position_end = lanelet.center_vertices[-1]

            # Transfrom the stop line position in curviliniear coordinates
            if not point_in_projection_domain(stop_line_position_0, coordinate_system) and not point_in_projection_domain(stop_line_position_end, coordinate_system):
                self._logger.warning("Lanlet id: " + str(lanelet_id) + " is not in the projection domain!")
                self._logger.warning("Stop line position is not in the projection domain! Stop line position: " + str(stop_line_position_0) + " and " + str(stop_line_position_end))
                continue
            elif not point_in_projection_domain(stop_line_position_0, coordinate_system):
                stop_line_position = stop_line_position_end
            elif not point_in_projection_domain(stop_line_position_end, coordinate_system):
                stop_line_position = stop_line_position_0
            else:
                stop_line_position_0_curv = coordinate_system.convert_to_curvilinear_coords(stop_line_position_0[0], stop_line_position_0[1])
                stop_line_position_end_curv = coordinate_system.convert_to_curvilinear_coords(stop_line_position_end[0], stop_line_position_end[1])
                # Check which stop line is closer to the vehicle
                if stop_line_position_0_curv[0] < stop_line_position_end_curv[0]:
                    stop_line_position = stop_line_position_0
                else:
                    stop_line_position = stop_line_position_end

            stop_line_position_curvilinear = coordinate_system.convert_to_curvilinear_coords(stop_line_position[0], stop_line_position[1])
            for traffic_light_id in lanelet.traffic_lights:
                if traffic_light_id in relevant_traffic_lights:
                    self._logger.warning("Traffic light is already assigned to another lanelet! Traffic light id: " + str(traffic_light_id)
                                     + ", lanelet id: " + str(lanelet_id))
                    # Check which stop line is closer to the vehicle
                    stop_line_position_current = relevant_traffic_lights[traffic_light_id]
                    stop_line_position_new = stop_line_position_curvilinear
                    # Check which stop line is closer to the vehicle
                    if stop_line_position_current[0] < stop_line_position_new[0]:
                        # Continue with the current stop line, if the current stop line is closer to the vehicle
                        continue

                relevant_traffic_lights[traffic_light_id] = stop_line_position_curvilinear

        self.inputs.relevant_traffic_lights = relevant_traffic_lights
        self._logger.debug("Relevant Traffic Lights: " + str(relevant_traffic_lights))

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating TrafficLightUpdate to " + str(new_status))

class TrafficLightOutOfRangeCondition(TrafficLightBehavior):
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
            # TODO: Take the reference path with curviliniear coordinates into account and check the distance
            # between the current position and the stop line position
            # TODO: For now, we assume that there is max one traffic light in range
            # Get nearest traffic light:
            min_distance_id = None
            min_distance = None

            distance = current_position[0] - stop_line_position[0]
            if distance < traffic_light_perception_range:
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    min_distance_id = traffic_light_id
                traffic_lights_in_range[traffic_light_id] = stop_line_position
            


        # Check if there are traffic lights in range
        # If there are traffic lights in range, save them in the blackboard and return FAILURE
        # If there are no traffic lights in range, return SUCCESS
        if len(traffic_lights_in_range) > 0:
            # TODO: For now, we assume that there is max one traffic light in range
            self.inputs.traffic_lights_in_range = {min_distance_id: traffic_lights_in_range[min_distance_id]}
            self.inputs.current_traffic_light_id = min_distance_id
            return Status.FAILURE
        else:
            # if no traffic light is in range, output the input velocity profile
            self.outputs.velocity_profile = self.inputs.velocity_profile_without_traffic_lights
            return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("Terminating TrafficLightOutOfRangeCondition to " + str(new_status))        


class YellowLightCondition(TrafficLightBehavior):
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
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating YellowLightCondition to " + str(new_status))


class RedLightCondition(TrafficLightBehavior):
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


class StopPointCalculationAction(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)


    def setup(self):
        self._logger.debug("Setting up StopPointCalculationAction")

    def initialise(self):
        self._logger.debug("Initialising StopPointCalculationAction")

    def update(self):
        self._logger.debug("Updating StopPointCalculationAction")

        traffic_lights_in_range: Dict[int, np.ndarray] = self.inputs.traffic_lights_in_range
        traffic_light_id = self.inputs.current_traffic_light_id
        # Get the stop line position of the traffic light
        stop_line_position = traffic_lights_in_range[traffic_light_id]

        # Get the current position of the vehicle
        current_position_curvilinear = self.global_inputs.current_position_curvilinear
        current_velocity = self.global_inputs.current_state.velocity
        self._logger.debug("Current Velocity: " + str(current_velocity))

        # Calculate the distance between the current position and the stop line position
        # Also consider the parameter for distance to stop line
        distance_to_stop_line = self.params.distance_to_stop_line
        distance = (stop_line_position[0] - distance_to_stop_line - current_position_curvilinear[0])

        self._logger.debug("Distance to Stop Line: " + str(distance))
        # Calculate the braking distance
        braking_distance = (current_velocity ** 2) / (2 * self.params.max_comfort_deceleration)
        self._logger.debug("Braking Distance: " + str(braking_distance))
        # Calculate the stop point
        stop_point = current_position_curvilinear[0] + (distance - braking_distance)
        if distance < 0.0:
            # Vehicle already passed the stop Line
            # check if the stop point is behind the stop line, without distance to stop line parameter
            distance_to_stop_line = (stop_line_position[0] - current_position_curvilinear[0])
            if distance_to_stop_line < -5.0:
                # Vehicle already passed the stop line
                # Go into Error Handling
                # TODO: Add Error Handling
                return Status.FAILURE
            stop_point_ahead = False

        elif distance >= braking_distance:
            # Vehicle has not reached the stop point yet
            # Breaking distance is smaller than the distance to the stop line
            stop_point_ahead = True
        else:
            # Vehicle already reached the stop point
            # Breaking distance is greater than the distance to the stop line
            stop_point_ahead = False

        # Save the hold position in the blackboard
        self.inputs.target_stop_position = stop_line_position[0] - distance_to_stop_line
        # Save the stop point in the blackboard
        self.inputs.stop_point = stop_point
        self.inputs.stop_point_ahead = stop_point_ahead
        

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating StopPointCalculationAction to " + str(new_status))


class StopPointAheadCondition(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    
    def setup(self):
        self._logger.debug("Setting up StopPointAheadCondition")

    def initialise(self):
        self._logger.debug("Initialising StopPointAheadCondition")

    def update(self):
        self._logger.debug("Updating StopPointAheadCondition")

        #TODO: When vehicle stops, it is behind the stop point, but should not continue driving
        # Check if the stop point is ahead of the vehicle
        # If the stop point is ahead of the vehicle, return SUCCESS
        # If curren velocity is low, also return SUCCESS
        current_velocity = self.global_inputs.current_state.velocity
        comfort_point_velocity = 3.0
        stop_point_ahead = self.inputs.stop_point_ahead
        if stop_point_ahead:
            return Status.SUCCESS
        elif current_velocity <= comfort_point_velocity:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating StopPointAheadCondition to " + str(new_status))


class StopPointBehindCondition(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    
    def setup(self):
        self._logger.debug("Setting up StopPointBehindCondition")

    def initialise(self):
        self._logger.debug("Initialising StopPointBehindCondition")

    def update(self):
        self._logger.debug("Updating StopPointBehindCondition")

        stop_point_ahead = self.inputs.stop_point_ahead
        if not stop_point_ahead:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("Terminating StopPointBehindCondition to " + str(new_status))


class EmergencyBrakingAction(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    
    def setup(self):
        self._logger.debug("Setting up EmergencyBrakingAction")

    def initialise(self):
        self._logger.debug("Initialising EmergencyBrakingAction")

    def update(self):
        self._logger.debug("Updating EmergencyBrakingAction")

        # TODO: WIP, this is comfort braking, not emergency braking

        current_position = self.global_inputs.current_position_curvilinear
        current_velocity = self.global_inputs.current_state.velocity
        target_stop_position = self.inputs.target_stop_position

        distance_to_stop_line = np.linalg.norm(target_stop_position - current_position)
        # Now calculate the new velocity profile with the comfort braking
        # for current position the current velocity is set, for the stop point the velocity is set to zero
        # in between we use a linear deceleration
        # Get the index of the current position in the input path
        input_path_curvilinear = self.global_inputs.input_path_curvilinear
        current_position_index = np.argmin(np.abs(input_path_curvilinear - current_position[0]))
        # Get the index of the stop point in the input path
        stop_point_index = np.argmin(np.abs(input_path_curvilinear - target_stop_position))
        # Define the new velocity profile
        # TODO: WHICH VELOCITY PROFILE SHOULD BE USED?
        velocity_profile = self.inputs.velocity_profile_without_traffic_lights
        # Set the velocity profile for the current position
        # TODO:
        # velocity_profile[current_position_index] = current_velocity
        # # Linear deceleration between current position and stop point
        # for i in range(current_position_index + 1, stop_point_index):
        #     distance_to_stop_point = np.linalg.norm(input_path_curvilinear[i] - target_stop_position)
        #     velocity_profile[i] = current_velocity - (current_velocity / distance_to_stop_line) * distance_to_stop_point
        # Set the velocity profile for the stop point and all points behind to zero

        # Set the velocity profile for all points zero
        velocity_profile[current_position_index:] = 0.0

        self._logger.debug("Velocity Profile: " + str(velocity_profile))

        self.outputs.velocity_profile = velocity_profile

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating EmergencyBrakingAction to " + str(new_status))


class ContinueDrivingAction(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    
    def setup(self):
        self._logger.debug("Setting up ContinueDrivingAction")

    def initialise(self):
        self._logger.debug("Initialising ContinueDrivingAction")

    def update(self):
        self._logger.debug("Updating ContinueDrivingAction")
        self.outputs.velocity_profile = self.inputs.velocity_profile_without_traffic_lights
        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating ContinueDrivingAction to " + str(new_status))


class ComfortBrakingAction(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    
    def setup(self):
        self._logger.debug("Setting up ComfortBrakingAction")

    def initialise(self):
        self._logger.debug("Initialising ComfortBrakingAction")

    def update(self):
        self._logger.debug("Updating ComfortBrakingAction")

        current_position = self.global_inputs.current_position_curvilinear
        current_velocity = self.global_inputs.current_state.velocity
        target_stop_position = self.inputs.target_stop_position

        distance_to_stop_line = np.linalg.norm(target_stop_position - current_position)
        # Now calculate the new velocity profile with the comfort braking
        # for current position the current velocity is set, for the stop point the velocity is set to zero
        # in between we use a linear deceleration
        # Get the index of the current position in the input path
        input_path_curvilinear = self.global_inputs.input_path_curvilinear
        current_position_index = np.argmin(np.abs(input_path_curvilinear - current_position[0]))
        # Get the index of the stop point in the input path
        stop_point_index = np.argmin(np.abs(input_path_curvilinear - target_stop_position))
        # Define the new velocity profile
        # TODO: WHICH VELOCITY PROFILE SHOULD BE USED?
        velocity_profile = self.inputs.velocity_profile_without_traffic_lights
        # Set the velocity profile for the current position
        # TODO:
        # velocity_profile[current_position_index] = current_velocity
        # # Linear deceleration between current position and stop point
        # for i in range(current_position_index + 1, stop_point_index):
        #     distance_to_stop_point = np.linalg.norm(input_path_curvilinear[i] - target_stop_position)
        #     velocity_profile[i] = current_velocity - (current_velocity / distance_to_stop_line) * distance_to_stop_point
        # Set the velocity profile for the stop point and all points behind to zero
        comfort_point_index = stop_point_index - 20
        comfort_point_velocity = 3.0

        velocity_profile[comfort_point_index:stop_point_index] = comfort_point_velocity

        velocity_profile[stop_point_index:] = 0.0

        self._logger.debug("Velocity Profile: " + str(velocity_profile))

        self.outputs.velocity_profile = velocity_profile

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating ComfortBrakingAction to " + str(new_status))
    

class PublishRVIZMarker(TrafficLightBehavior):
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
            stop_line_curv = self.inputs.target_stop_position
            stop_point_curv = self.inputs.stop_point

            stop_line = coordinate_system.convert_to_cartesian_coords(stop_line_curv, 0.0)
            self._logger.debug("Stop Line: " + str(stop_line))
            stop_point = coordinate_system.convert_to_cartesian_coords(stop_point_curv, 0.0)
            self._logger.debug("Stop Point: " + str(stop_point))

        except:
            stop_line = None
            stop_point = None

        z = self.global_inputs.get("z_coordinate")
        
        positions = [stop_line, stop_point]
        text = ["StopLine", "StopPoint"]
        positions_aw = []
        # convert positions to AW coordinate system
        for pos in positions:
            if pos is None:
                continue
            positions_aw.append(utm2map(self.global_inputs.get("origin_transformation"), pos))

        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.global_inputs.get("current_time_msg")
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
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
        for pos in positions_aw:
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
            
        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("Terminating PublishRVIZMarker to " + str(new_status))


class ErrorHandlingAction(TrafficLightBehavior):
    def __init__(self, name, logger: RcutilsLogger):
        super().__init__(name, logger)

    # TODO: Implement Error Handling

    def update(self):
        # When Traffic Light is inactive, or other errors occur, the vehicle should continue driving (e.g. car stops behind the stop line)

        # When in Error State, velocity profile is not changed
        self.outputs.velocity_profile = self.inputs.velocity_profile_without_traffic_lights

    
        # Delete all markers
        marker_array = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)
        self.outputs.traffic_light_marker_array = marker_array
        return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("Terminating ErrorHandlingAction to " + str(new_status))
