import py_trees
import math
from ...base.base_tree import BaseTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector
from commonroad.scenario.scenario import Scenario
from cr2autoware.common.configuration import BehaviorPlannerParams
from cr2autoware.common.configuration import CR2AutowareParams
from cr2autoware.interfaces.implementation.behavior_tree.behavior_utils import minimum_width_lanelet
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from typing import List, Set, Dict
import copy
import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from abc import abstractmethod
from visualization_msgs.msg import MarkerArray, Marker
from cr2autoware.common.utils.transform import utm2map
from geometry_msgs.msg import Point as PointMsg



class LaneKeepingTree(BaseTree):
    """
    Submodule for lane keeping handling.

    :var logger: ROS2 node logger
    :var verbose: Flag for verbose logging
    :var root: Root node of the behavior tree
    """
    def __init__(self, logger: RcutilsLogger, verbose: bool):
        super(LaneKeepingTree, self).__init__(logger, verbose)
        self.root = self.create_behavior_tree()

    def create_behavior_tree(self):
        root = Selector(name="LaneKeepingModule", memory=False)
        
        # Initialize Sub Module
        lane_keeping_sequence = Sequence(name="LaneKeepingSequence", memory=False)

        check_for_restrictions = Selector(name="CheckForRestrictions", memory=False)

        blackboard_condition = BlackboardCondition(name="BlackboardCondition", logger=self.logger)
        ros_condition = ROSCondition(name="ROSCondition", logger=self.logger)
        config_condition = ConfigCondition(name="ConfigCondition", logger=self.logger)

        apply_lane_keeping = ApplyLaneKeepingAction(name="ApplyLaneKeepingAction", logger=self.logger)

        publish_rviz_marker = PublishRVIZMarker(name="PublishRVIZMarkerLaneKeeping", logger=self.logger)

        no_lane_keeping_sequence = Sequence(name="NoLaneKeepingSequence", memory=False)

        apply_default_action = ApplyDefaultAction(name="ApplyDefaultAction", logger=self.logger)

        publish_rviz_marker_no_lane_keeping = PublishRVIZMarker(name="PublishRVIZMarkerNoLaneKeeping", logger=self.logger)

        # Add children to the tree
        check_for_restrictions.add_children([blackboard_condition, ros_condition, config_condition])
        lane_keeping_sequence.add_children([check_for_restrictions, apply_lane_keeping, publish_rviz_marker])
        no_lane_keeping_sequence.add_children([apply_default_action, publish_rviz_marker_no_lane_keeping])

        root.add_children([lane_keeping_sequence, no_lane_keeping_sequence])

        return root

class LaneKeepingBehavior(Behaviour):
    """
    Base class for lane keeping behavior.

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
        self.inputs = py_trees.blackboard.Client(name=(name + "Inputs"), namespace="/modules/lane_keeping/inputs")
        self.inputs.register_key("blackboard_condition", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("ros_condition", access=py_trees.common.Access.WRITE)

        # Register keys for Module Outputs
        self.outputs = py_trees.blackboard.Client(name=(name + "Outputs"), namespace="/modules/lane_keeping/outputs")
        self.outputs.register_key("lane_keeping_marker_array", access=py_trees.common.Access.WRITE)
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


class ApplyLaneKeepingAction(LaneKeepingBehavior):
    """
    Action Node. Calculates lane keeping restriction parameters.

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
        #TODO: Check current position index for cartesian input path + index lenght
        reference_path: np.ndarray = self.global_inputs.input_path
        current_position_index: int = self.global_inputs.current_position_index

        max_relevant_index = current_position_index + math.ceil(self.params.min_look_ahead_distance_lane_width) + 1
        relevant_path = reference_path[current_position_index:max_relevant_index]

        path: List[np.ndarray] = [np.array(p) for p in relevant_path]
        relevant_lanelets_nested_list: List[List[int]] = scenario.lanelet_network.find_lanelet_by_position(path)
        # Transform the lanelet ids from the nested list to a set
        relevant_lanelets: Set[int] = set()
        for lanelet_id_list in relevant_lanelets_nested_list:
            for lanelet_id in lanelet_id_list:
                relevant_lanelets.add(lanelet_id)

        # Get minimal width of the lanelets
        min_width = None
        for lanelet_id in relevant_lanelets:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            width = minimum_width_lanelet(lanelet)

            if min_width is None or width < min_width:
                min_width = width
        
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

        # Apply limits if d_abs is smaller than the default d_min and d_max
        if d_abs < np.abs(default_d_min):
            self.outputs.d_min = -d_abs
        else:
            self.outputs.d_min = None
        
        if d_abs < np.abs(default_d_max):
            self.outputs.d_max = d_abs
        else:
            self.outputs.d_max = None

        return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ApplyLaneKeepingAction to " + str(new_status))        


class BlackboardCondition(LaneKeepingBehavior):
    """
    Condition Node. Checks if lane keeping is required by the blackboard parameter.

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
        self.inputs.blackboard_condition = False

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        blackboard_condition = self.inputs.blackboard_condition

        if blackboard_condition:
            # reset the condition
            self.inputs.blackboard_condition = False
            return Status.SUCCESS
        else:
            return Status.FAILURE

        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating BlackboardCondition to " + str(new_status))


class ROSCondition(LaneKeepingBehavior):
    """
    Condition Node. Checks if lane keeping is required by the ROS2 topic.

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
        self.inputs.ros_condition = False

    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        ros_condition = self.inputs.ros_condition

        if ros_condition:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ROSCondition to " + str(new_status))


class ConfigCondition(LaneKeepingBehavior):
    """
    Condition Node. Checks the configuration force_lane_keeping parameter.

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
        config_condition = self.params.force_lane_keeping
        if config_condition:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ConfigCondition to " + str(new_status))


class ApplyDefaultAction(LaneKeepingBehavior):
    """
    Action Node. Applies the default parameters for no lane keeping.

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
        # Set the default values for d_min and d_max
        self.outputs.d_min = None
        self.outputs.d_max = None
        return Status.SUCCESS

    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating ApplyDefaultAction to " + str(new_status))
    

class PublishRVIZMarker(LaneKeepingBehavior):
    """
    Visualization Node for RVIZ. Publishes 

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

        if self.params.publish_lane_keeping_markers:
            coordinate_system: CoordinateSystem = self.global_inputs.get("coordinate_system")
            z = self.global_inputs.get("z_coordinate")

            # Create a marker array
            marker_array = MarkerArray()
            del_marker = Marker()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)

            if self.outputs.exists("d_min") and self.outputs.exists("d_max"):
                if self.outputs.d_min is not None and self.outputs.d_max is not None:

                    input_path_curvilinear = self.global_inputs.input_path_curvilinear
                    current_position_index = self.global_inputs.current_position_index
                    max_relevant_index = current_position_index + math.ceil(self.params.min_look_ahead_distance_lane_width) + 1
                    coordinate_system: CoordinateSystem = self.global_inputs.coordinate_system

                    path = input_path_curvilinear[current_position_index:max_relevant_index]

                    d_min_vehicle = self.outputs.d_min - 0.5 * (self.global_params.vehicle.wheel_tread + self.global_params.vehicle.right_overhang + self.global_params.vehicle.left_overhang)
                    d_max_vehicle = self.outputs.d_max + 0.5 * (self.global_params.vehicle.wheel_tread + self.global_params.vehicle.right_overhang + self.global_params.vehicle.left_overhang)
                    
                    min_path: List[np.ndarray] = []
                    max_path: List[np.ndarray] = []
                    for point in path:
                        point_cart_min = coordinate_system.convert_to_cartesian_coords(point, d_min_vehicle)
                        try:
                            point_cart_min_aw = utm2map(self.global_inputs.get("origin_transformation"), point_cart_min)
                            point_cart_min_aw_msg = PointMsg(x=point_cart_min_aw.x, y=point_cart_min_aw.y, z=z)
                            min_path.append(point_cart_min_aw_msg)
                        except:
                            pass
                        point_cart_max = coordinate_system.convert_to_cartesian_coords(point, d_max_vehicle)
                        try:
                            point_cart_max_aw = utm2map(self.global_inputs.get("origin_transformation"), point_cart_max)
                            point_cart_max_aw_msg = PointMsg(x=point_cart_max_aw.x, y=point_cart_max_aw.y, z=z)
                            max_path.append(point_cart_max_aw_msg)
                        except:
                            pass

                    no_overtake_marker = Marker()
                    no_overtake_marker.header.frame_id = "map"
                    no_overtake_marker.header.stamp = self.global_inputs.get("current_time_msg")
                    no_overtake_marker.type = Marker.LINE_STRIP
                    no_overtake_marker.action = Marker.ADD
                    no_overtake_marker.scale.x = 0.1
                    no_overtake_marker.color.a = 1.0
                    no_overtake_marker.color.r = 1.0
                    no_overtake_marker.color.g = 1.0
                    no_overtake_marker.color.b = 1.0
                    
                    no_overtake_marker_min = copy.deepcopy(no_overtake_marker)
                    no_overtake_marker_max = copy.deepcopy(no_overtake_marker)

                    no_overtake_marker_min.id = 11
                    no_overtake_marker_min.ns = "no_overtake_min"
                    no_overtake_marker_min.points = min_path
                    
                    no_overtake_marker_max.id = 12
                    no_overtake_marker_max.ns = "no_overtake_max"
                    no_overtake_marker_max.points = max_path


                    marker_array.markers.append(no_overtake_marker_min)
                    marker_array.markers.append(no_overtake_marker_max)

        self.outputs.lane_keeping_marker_array = marker_array

        return Status.SUCCESS
        
    def terminate(self, new_status):
        self._logger.debug("[SVEN]Terminating PublishRVIZMarker to " + str(new_status))
