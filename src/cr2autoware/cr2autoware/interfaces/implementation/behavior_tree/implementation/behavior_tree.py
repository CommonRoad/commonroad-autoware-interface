from ..base.base_tree import BaseTree
import numpy as np
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector, Parallel
from .modules.traffic_light_behavior import TrafficLightsTree
from commonroad.scenario.scenario import Scenario
from  cr2autoware.common.configuration import BehaviorPlannerParams
from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from typing import List
from rclpy.time import Time
from rclpy.impl.rcutils_logger import RcutilsLogger

class BehaviorTree(BaseTree):

    def __init__(self, logger: RcutilsLogger, verbose: bool, config=None):
        super(BehaviorTree, self).__init__(logger, verbose, config)
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
        self.inputs.register_key("z_coordinate", access=py_trees.common.Access.WRITE)
        self.inputs.register_key("current_time_msg", access=py_trees.common.Access.WRITE)

        # Register keys for Outputs
        self.outputs.register_key("velocity_profile", access=py_trees.common.Access.WRITE)

        # Register keys for Modules
        self.blackboard.register_key("/modules/traffic_lights/inputs/velocity_profile_without_traffic_lights", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("/modules/traffic_lights/outputs/velocity_profile", access=py_trees.common.Access.READ)

        # After initialization, create the behavior tree
        self.root = self.create_behavior_tree()

        # show the tree in the log
        self._printed_tree = False
        self.output_tree_in_log()


    @property
    def velocity_profile(self):
        return self.outputs.velocity_profile

    def default_config(self):
        return {
            "root": {
                "type": "Sequence",
                "name": "MainBehaviorTree",
                "children": [
                    {"type": "TrafficLightTree"}
                    # Add more sub-trees or behaviors here
                ]
            }
        }

    def create_behavior_tree(self):
        root = Parallel("MainBehaviorTree", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True))

        # Initialize Sub Modules
        self.traffic_light_module = TrafficLightsTree(self.logger, self.verbose)
        
        # Add sub-trees or behaviors here
        root.add_child(self.traffic_light_module.root)
        
        return root

    # def init_parameters(self, params: BehaviorPlannerParams):

    #     # Init Traffic Light Parameters
    #     self.traffic_light_params = py_trees.blackboard.Client(name="TrafficLightParams", namespace="/modules/traffic_lights/params")
    #     self.traffic_light_params.register_key("traffic_light_perception_range", access=py_trees.common.Access.WRITE)
    #     self.traffic_light_params.traffic_light_perception_range = params.traffic_light_perception_range

    def preprocessing(self, scenario: Scenario, current_state: EgoVehicleState, input_path: np.ndarray, coordinate_system: CoordinateSystem, input_path_curvilinear: np.ndarray, origin_transformation: List, z_coordinate: float, ros_time_msg: Time) -> None:
        # TODO: Match last input path with current input_path and match the velocity profile, 
        # TODO: so that both paths and profiles are aligned and can be compared in the behavior tree
        self.inputs.scenario = scenario = scenario
        self.inputs.current_state = current_state
        self.inputs.input_path = input_path
        self.inputs.coordinate_system = coordinate_system
        self.inputs.input_path_curvilinear = input_path_curvilinear
        self.inputs.origin_transformation = origin_transformation
        self.inputs.current_position_curvilinear = coordinate_system.convert_to_curvilinear_coords(current_state.position[0], current_state.position[1])
        self.inputs.z_coordinate = z_coordinate
        self.inputs.current_time_msg = ros_time_msg

        # save the last velocity profile
        if self.outputs.exists("velocity_profile"):
            # Check if the planning cycle is new
            if len(input_path) == len(self.outputs.velocity_profile):
                self.logger.debug("Length of the input path and the last velocity profile are equal")
                new_planning_cycle = False
            else:
                self.logger.debug("Length of the input path and the last velocity profile are not equal")
                new_planning_cycle = True

            if not new_planning_cycle:
                self.logger.debug("Velocity profile from the last planning cycle is available")
                self.inputs.last_velocity_profile = self.outputs.velocity_profile
            else:
                self.logger.debug("Velocity profile from the last planning cycle is not available")
                self.inputs.last_velocity_profile = None

        else:
            self.logger.debug("No velocity profile available from the last planning cycle")
            self.inputs.last_velocity_profile = None

        # Get the velocity profile without traffic lights
        # TODO: For Concept create array of lenght of the path and fill it with 20 m/s
        velocity_profile_without_traffic_lights = np.full(len(input_path), 20.0)

        # Create velocity profile for traffic lights
        # Velocity profile includes all modules that influence the velocity profile but the traffic lights module
        self.blackboard.modules.traffic_lights.inputs.velocity_profile_without_traffic_lights = velocity_profile_without_traffic_lights


    def plan(self) -> None:
        """
        """
        # Tick the behavior tree
        self.tick_once()


    
    def prepare_output(self) -> np.ndarray:
        
        self._update_velocity_profile()

        # self.output_tree_in_log()

        # Return the velocity profile
        return self.outputs.velocity_profile

    def _update_velocity_profile(self):
        if self.blackboard.exists("/modules/traffic_lights/outputs/velocity_profile"):
            global_velocity_profile = self.blackboard.modules.traffic_lights.outputs.velocity_profile

            self.outputs.velocity_profile = global_velocity_profile
        else:
            self.outputs.velocity_profile = np.full(len(self.inputs.input_path), 20.0)
    
    def output_tree_in_log(self):
        #self.logger.debug(py_trees.display.unicode_tree(self.root, show_status=True))
        self.logger.debug(py_trees.display.unicode_blackboard())
        # self.logger.debug(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
        # self.logger.debug(py_trees.display.unicode_blackboard_activity_stream())
        # Only for debugging purposes
        # if not self._printed_tree:
        #     parent_directory = '/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/cr2autoware/interfaces/implementation/behavior_tree'
        #     py_trees.display.render_dot_tree(self.root,
        #                                     visibility_level=py_trees.common.VisibilityLevel.DETAIL,
        #                                     name='behavior_tree', 
        #                                     target_directory=parent_directory,
        #                                     )
        #     self._printed_tree = True
