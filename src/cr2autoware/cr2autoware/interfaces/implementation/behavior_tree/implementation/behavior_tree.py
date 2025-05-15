from ..base.base_tree import BaseTree
import numpy as np
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from .failsafe import FailSafe
from .modules.traffic_light_behavior import TrafficLightsTree
from .modules.lateral_clearance_velocity_adjuster import LateralClearanceVelocityAdjusterTree
from .modules.lane_keeping import LaneKeepingTree
from commonroad.scenario.scenario import Scenario
from  cr2autoware.common.configuration import BehaviorPlannerParams
from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from typing import List
from rclpy.time import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from ..behavior_utils import copy_from_blackboard, calculate_current_position_index

class BehaviorTree(BaseTree):

    def __init__(self, logger: RcutilsLogger, verbose: bool):
        """
        Main behavior tree for the behavior planning.

        The behavior tree consits of various behavior modules that influence the velocity profile and lateral offset (d) for trajectory planning. 
        It is built and managed using the py_trees package.

        ----------------
        **Behavior Modules:**

        * Traffic lights module:
            * Class: TrafficLightsTree
            * Description: The module calculates the velocity profile for the input path considering the traffic lights.
        * Lateral clearance velocity adjuster module:
            * Class: LateralClearanceVelocityAdjusterTree
            * Description: The module adjusts the velocity profile considering the lateral clearance function.

        ----------------
        :var logger: ROS2 node logger
        :var verbose: Flag for verbose logging
        :var blackboard: Blackboard for the behavior tree
        :var inputs: Blackboard client for inputs
        :var outputs: Blackboard client for outputs
        :var traffic_light_module: Traffic lights module
        :var lateral_clearance_velocity_adjuster: Lateral clearance velocity adjuster module
        :var root: Root node of the behavior tree
        :var _printed_tree: Flag for printing the tree in the log
        :var new_planning_cycle: Flag for a new planning cycle
        """
        super(BehaviorTree, self).__init__(logger, verbose)
        # Initialize the blackboard
        # Create blackboard clients
        self.blackboard = py_trees.blackboard.Client(name="GlobalBehaviorTreeBlackboard")
        self.inputs = py_trees.blackboard.Client(name="GlobalBehaviorTreeInputs", namespace="inputs")
        self.outputs = py_trees.blackboard.Client(name="GlobalBehaviorTreeOutputs", namespace="outputs")

        # Register keys for Inputs
        self.inputs.register_key("scenario", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("current_state", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("input_path", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("coordinate_system", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("input_path_curvilinear", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("origin_transformation", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("last_velocity_profile", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("current_position_curvilinear", access=py_trees.common.Access.WRITE)
        self.inputs.current_position_curvilinear = None
        self.inputs.register_key("z_coordinate", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("current_time_msg", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("current_position_index", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("input_path_orientation", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("empty_velocity_profile", access=py_trees.common.Access.WRITE)

        # Register keys for Outputs
        self.outputs.register_key("velocity_profile", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("d_min", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("d_max", access=py_trees.common.Access.WRITE)

        # Initialize necessary outputs
        self.outputs.d_min = None
        self.outputs.d_max = None

        # Register keys for Modules
        self.blackboard.register_key("/modules/traffic_lights/inputs/velocity_profile_without_traffic_lights", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("/modules/traffic_lights/outputs/velocity_profile", access=py_trees.common.Access.READ)
        self.blackboard.register_key("/modules/lane_keeping/outputs/d_min", access=py_trees.common.Access.READ)
        self.blackboard.register_key("/modules/lane_keeping/outputs/d_max", access=py_trees.common.Access.READ)
        self.blackboard.register_key("/modules/lateral_clearance/outputs/velocity_profile", access=py_trees.common.Access.READ)
        self.blackboard.register_key("params", access=py_trees.common.Access.READ)
        
        # Get the parameters
        self.params: BehaviorPlannerParams = self.blackboard.params

        # After initialization, create the behavior tree
        self.root = self.create_behavior_tree()

        # show the tree in the log
        self._printed_tree = False
        self.output_tree_in_log()

        # new planning cycle
        self.new_planning_cycle = True

    @property
    def velocity_profile(self):
        """
        Velocity profile of the input path. 
        
        The velocity profile is calculated by the behavior tree and can be passed to the AW velocity smoother.
        :return: Velocity profile of the input path
        """
        return copy_from_blackboard(self.outputs.velocity_profile)

    def create_behavior_tree(self):
        """
        Create the behavior tree for the behavior planner.

        Add the sub-trees or behaviors here.

        :return: Behavior tree root
        """
        root = Selector(name="MainBehaviorTree", memory=False)

        # Create the main behavior tree
        fail_safe = FailSafe(name="FailSafe", logger=self.logger)
        module_tree = Parallel(name="BehaviorModules", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True))
        
        # Initialize Sub Modules
        self.traffic_light_module = TrafficLightsTree(self.logger, self.verbose)
        self.lateral_clearance_velocity_adjuster = LateralClearanceVelocityAdjusterTree(self.logger, self.verbose)
        self.lane_keeping_module = LaneKeepingTree(self.logger, self.verbose)
        
        # Add sub-trees or behaviors here
        if self.params.traffic_light_behavior:
            module_tree.add_child(self.traffic_light_module.root)
        if self.params.lateral_clearance_velocity_adjuster:
            module_tree.add_child(self.lateral_clearance_velocity_adjuster.root)
        if self.params.lane_keeping:
            module_tree.add_child(self.lane_keeping_module.root)
        
        root.add_children([module_tree, fail_safe])
        return root

    def preprocessing(self, scenario: Scenario, current_state: EgoVehicleState, input_path: np.ndarray, coordinate_system: CoordinateSystem, input_path_curvilinear: np.ndarray, origin_transformation: List, z_coordinate: float, ros_time_msg: Time, input_path_orientation: np.ndarray) -> None:
        """
        Preprocess the input data for the behavior planner.

        All necessary data is stored in the blackboard and can be accessed by the behavior tree. Also, data from the last planning cycle is processed and stored in the blackboard.

        :param scenario: CommonRoad scenario
        :param current_state: Current state of the ego vehicle
        :param input_path: Input path for the behavior planner
        :param coordinate_system: Coordinate system
        :param input_path_curvilinear: Input path in curvilinear coordinates
        :param origin_transformation: Origin transformation
        :param z_coordinate: Z coordinate
        :param ros_time_msg: ROS time message
        :param input_path_orientation: Input path orientation
        """
        # Store the input data in the blackboard
        self.inputs.scenario = scenario = scenario
        self.inputs.current_state = current_state
        self.inputs.input_path = input_path
        self.inputs.coordinate_system = coordinate_system
        self.inputs.input_path_curvilinear = input_path_curvilinear
        self.inputs.origin_transformation = origin_transformation
        current_position_curvilinear = coordinate_system.convert_to_curvilinear_coords(current_state.position[0], current_state.position[1])
        self.inputs.current_position_curvilinear = current_position_curvilinear
        self.inputs.z_coordinate = z_coordinate
        self.inputs.current_time_msg = ros_time_msg
        self.inputs.current_position_index = calculate_current_position_index(current_position_curvilinear, input_path_curvilinear)
        self.inputs.input_path_orientation = input_path_orientation
        self.inputs.empty_velocity_profile = np.full(len(input_path), float("inf"))

        # Preprocess the data from the last planning cycle:

        # save the last velocity profile
        if self.outputs.exists("velocity_profile"):
            # Check if the planning cycle is new
            if len(input_path) == len(self.outputs.velocity_profile):
                self.logger.debug("Length of the input path and the last velocity profile are equal")
                self.new_planning_cycle = False
            else:
                self.logger.debug("Length of the input path and the last velocity profile are not equal")
                self.new_planning_cycle = True

            if not self.new_planning_cycle:
                self.logger.debug("Velocity profile from the last planning cycle is available")
                self.inputs.last_velocity_profile = copy_from_blackboard(self.outputs.velocity_profile)
            else:
                self.logger.debug("Velocity profile from the last planning cycle is not available")
                self.inputs.last_velocity_profile = None

        else:
            self.logger.debug("No velocity profile available from the last planning cycle")
            self.inputs.last_velocity_profile = None

        # Create velocity profile for traffic lights
        # Velocity profile includes all modules that influence the velocity profile but the traffic lights module
        self.blackboard.modules.traffic_lights.inputs.velocity_profile_without_traffic_lights = self._create_velocity_profile(new_planning_cycle=self.new_planning_cycle, no_traffic_lights=True)

    def plan(self) -> None:
        """
        Behavior planner planning cycle.
        """
        # Tick the behavior tree
        self.tick_once()

    def prepare_output(self) -> None:
        """
        Prepare the output of the behavior planner.

        Update global output variables of the behavior planner and store them in the blackboard.
        """
        # Update the velocity profile
        self.outputs.velocity_profile = self._create_velocity_profile()

        # Update the lateral offset d
        self._update_lateral_offset_d()

        # For debugging purposes:
        self.output_tree_in_log()

    def _create_velocity_profile(self, new_planning_cycle: bool = False, no_traffic_lights: bool = False) -> np.ndarray:
        """
        Combine the velocity profiles from the different modules to one global velocity profile for the behavior planner.

        :param no_traffic_lights: If True, the velocity profile from the traffic lights module is not considered
        """
        check_profiles: List[np.ndarray] = []
        # skip velocity profile from modules we preprocess in a new planning cycle
        if not new_planning_cycle:
            if self.blackboard.exists("/modules/traffic_lights/outputs/velocity_profile") and not no_traffic_lights:
                #self.logger.debug(f"Traffic lights velocity profile: {(self.blackboard.modules.traffic_lights.outputs.velocity_profile)}")
                #self.logger.debug(f"Length of velocity profile from traffic lights: {len(self.blackboard.modules.traffic_lights.outputs.velocity_profile)}")
                check_profiles.append(self.blackboard.modules.traffic_lights.outputs.velocity_profile)
            if self.blackboard.exists("/modules/lateral_clearance/outputs/velocity_profile"):
                #self.logger.debug(f"Lateral clearance velocity profile: {(self.blackboard.modules.lateral_clearance.outputs.velocity_profile)}")
                #self.logger.debug(f"Length of velocity profile from lateral clearance: {len(self.blackboard.modules.lateral_clearance.outputs.velocity_profile)}")
                check_profiles.append(self.blackboard.modules.lateral_clearance.outputs.velocity_profile)

        velocity_profile = np.full(len(self.inputs.input_path), self.params.velocity_limit)
        #self.logger.debug("Velocity profile: " + str(velocity_profile))
        #self.logger.debug(f"Length of velocity profile: {len(velocity_profile)}")
        for profile in check_profiles:
            try:
                velocity_profile = np.minimum(velocity_profile, profile)
            except ValueError as e:
                self.logger.error(f"Error in combining velocity profiles: {e}")
                self.logger.error(f"Length of velocity profile: {len(velocity_profile)}")
                self.logger.error(f"Length of profile: {len(profile)}")
                self.logger.error(f"Velocity profile: {velocity_profile}")
                self.logger.error(f"Profile: {profile}")
                self.logger.error(f"Check profiles: {check_profiles}")
                raise e
        # self.logger.debug("Updated velocity profile: " + str(velocity_profile))

        return velocity_profile

    def _update_lateral_offset_d(self):
        """
        Update the lateral offset d in the blackboard. 

        This parameter is used in the reactive planner to calculate the minimum and maximum lateral offset d for trajectory planning.
        """

        if self.blackboard.exists("/modules/lane_keeping/outputs/d_min"):
            self.outputs.d_min = self.blackboard.modules.lane_keeping.outputs.d_min
        else:
            self.outputs.d_min = None

        if self.blackboard.exists("/modules/lane_keeping/outputs/d_max"):
            self.outputs.d_max = self.blackboard.modules.lane_keeping.outputs.d_max
        else:
            self.outputs.d_max = None
        
        self.logger.debug(f"[SVEN]Updated d_min: {self.outputs.d_min}")
        self.logger.debug(f"[SVEN]Updated d_max: {self.outputs.d_max}")
    
    def output_tree_in_log(self):
        """
        For Debugging purposes: Output the behavior tree in the log.
        """
        #self.logger.debug(py_trees.display.unicode_tree(self.root, show_status=True))
        # self.logger.debug(py_trees.display.unicode_blackboard())
        # self.logger.debug(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
        # self.logger.debug(py_trees.display.unicode_blackboard_activity_stream())
        # Only for debugging purposes
        if not self._printed_tree:
            parent_directory = '/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/cr2autoware/interfaces/implementation/behavior_tree/output/behavior_tree'
            py_trees.display.render_dot_tree(self.root,
                                            visibility_level=py_trees.common.VisibilityLevel.DETAIL,
                                            name='behavior_tree', 
                                            target_directory=parent_directory,
                                            )
            self._printed_tree = True
