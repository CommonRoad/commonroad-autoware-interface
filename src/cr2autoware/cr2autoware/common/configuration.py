"""
Configuration module for CR2Auto node.
======================================

This module defines the parameter structure for the CR2Auto node.

All ROS parameters are encapsulated within one dataclass CR2AutowareParams (with different sub-dataclasses) and are
declared and set upon initialization of the node.
---------------------------
"""

# typing
import typing
from typing import Any, List

# standard imports
from dataclasses import dataclass, field, fields
import yaml

# ROS imports
from rclpy.parameter import Parameter

# cr2autoware imports
from cr2autoware.common.utils.general import get_absolute_path_from_package

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


@dataclass
class BaseParams:
    """
    CR2Auto base parameters.

    :var _node: reference to CR2Auto ROS node
    """
    # reference to CR2Auto ROS node
    _node: "Cr2Auto"

    def __getitem__(self, item: str) -> Any:
        """
        Getter for base parameter value.

        :param item: parameter name
        """
        try:
            value = self.__getattribute__(item)
        except AttributeError as e:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}") from e
        return value

    def __setitem__(self, key: str, value: Any):
        """
        Setter for item.

        :param key: parameter name
        :param value: parameter value
        """
        try:
            self.__setattr__(key, value)
        except AttributeError as e:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}") from e

    def _declare_ros_params(self, namespace: str):
        """
        Function to declare ROS params in the associated ROS2 node (self._node).

        :param namespace: namespace for ROS parameters
        """
        for field_info in fields(self):
            # ignore _node field
            if not field_info.name.startswith('_'):
                key = field_info.name
                val = getattr(self, key)
                ros_param_name = namespace + "." + key
                self._node.declare_parameter(ros_param_name, val).value


@dataclass
class GeneralParams(BaseParams):
    """
    Class for general parameters.

    :var map_path: path to map directory (Launch arg)
    :var solution_file: path to CR solution file (Launch arg)
    :var detailed_log: verbose ROS logging
    :var write_scenario: write scenario to XML file
    :var plot_scenario: plot CR scenario
    :var store_trajectory: store driven trajectory as CR solution file
    :var store_trajectory_file: path to store CR solution file
    """
    # path to map directory (Launch arg)
    map_path: str = ""
    # path to CR solution file (Launch arg)
    solution_file: str = ""

    # verbose ROS logging
    detailed_log: bool = True
    # write scenario to XML file
    write_scenario: bool = True
    # plot CR scenario
    plot_scenario: bool = False
    # store driven trajectory as CR solution file
    store_trajectory: bool = False
    # path to store CR solution file
    store_trajectory_file: str = "output/solutions/solution1.xml"

    def __post_init__(self):
        """Initialize ROS params."""
        # declare ROS params
        self._declare_ros_params(namespace="general")


@dataclass
class ScenarioParams(BaseParams):
    """Class for scenario parameters"""
    # scenario time step
    dt: float = 0.1
    # settings for Lanelet2CR conversion
    left_driving: bool = True
    adjacencies: bool = False
    # publish obstacles from CR scenario
    publish_obstacles: bool = True

    def __post_init__(self):
        """Initialize ROS params."""
        # declare ROS params
        self._declare_ros_params(namespace="scenario")


@dataclass
class VehicleParams(BaseParams):
    """
    Class for handling vehicle parameters.

    * Static vehicle parameters are read from the vehicle_info.param.yaml file in the vehicle_description ROS package
    * Additional static (e.g., rear axle distance) and dynamic (e.g., max-velocity) parameters are defined here and
      are declared as ROS parameters for the CR2Auto node

    ---------------------------
    :var vehicle_info_param_file: full path to vehicle_info.param.yaml file
    :var wheel_base: wheel base of the vehicle
    :var wheel_tread: wheel tread of the vehicle
    :var front_overhang: front overhang of the vehicle
    :var rear_overhang: rear overhang of the vehicle
    :var left_overhang: left overhang of the vehicle
    :var right_overhang: right overhang of the vehicle
    :var max_steer_angle: maximum steering angle of the vehicle
    :var wb_front_axle: front axle distance to center of gravity
    :var wb_rear_axle: rear axle distance to center of gravity
    :var max_velocity: maximum velocity of the vehicle
    :var min_velocity: minimum velocity of the vehicle
    :var max_acceleration: maximum acceleration of the vehicle
    """
    # full path to vehicle_info.param.yaml file
    vehicle_info_param_file: str = ""

    # Static params: these are initialized here with default values, but are set afterwards from the
    # in the post_init method via vehicle_info.param.yaml file
    wheel_base: float = 3.128  # meter
    wheel_tread: float = 1.645  # meter
    front_overhang: float = 0.952  # meter
    rear_overhang: float = 0.897  # meter
    left_overhang: float = 0.304  # meter
    right_overhang: float = 0.304  # meter
    max_steer_angle: float = 0.610865  # rad

    # kinamatic single track params
    wb_front_axle: float = 1.484  # meter
    wb_rear_axle: float = 1.644  # meter

    # velocity limits (in m/s)
    max_velocity: float = 10.0
    min_velocity: float = 0.0
    max_acceleration: float = 10.0

    def __post_init__(self):
        """Initialize ROS params."""
        # declare ROS params
        self._declare_ros_params(namespace="vehicle")

        # Read vehicle info YAML file from ROS Params
        self.vehicle_info_param_file = \
            self._node.get_parameter("vehicle.vehicle_info_param_file").get_parameter_value().string_value
        with open(self.vehicle_info_param_file, "r") as f:
            vehicle_info_params = yaml.safe_load(f)["/**"]["ros__parameters"]

        # Set ROS params
        param_type_double = Parameter.Type.DOUBLE
        ros_vehicle_param_list: List[Parameter] = [
            Parameter("vehicle.wheel_base", param_type_double, vehicle_info_params["wheel_base"]),
            Parameter("vehicle.wheel_tread", param_type_double, vehicle_info_params["wheel_tread"]),
            Parameter("vehicle.front_overhang", param_type_double, vehicle_info_params["front_overhang"]),
            Parameter("vehicle.rear_overhang", param_type_double, vehicle_info_params["rear_overhang"]),
            Parameter("vehicle.left_overhang", param_type_double, vehicle_info_params["left_overhang"]),
            Parameter("vehicle.right_overhang", param_type_double, vehicle_info_params["right_overhang"]),
            Parameter("vehicle.max_steer_angle", param_type_double, vehicle_info_params["max_steer_angle"]),
        ]
        self._node.set_parameters(ros_vehicle_param_list)


@dataclass
class VelocityPlannerParams(BaseParams):
    """
    Class for velocity planner parameters.

    :var lookahead_dist: lookahead distance (in meters)
    :var lookahead_time: lookahead time (in seconds)
    :var init_velocity: initial velocity (in m/s) (TODO: not yet implemented -> remove?; should only be used for lanelet2 maps)
    """
    # lookahead distance (in meters)
    lookahead_dist: float = 3.0
    # lookahead time (in seconds)
    lookahead_time: float = 0.8
    # initial velocity (in m/s) (TODO: not yet implemented -> remove?; should only be used for lanelet2 maps)
    init_velocity: float = 5.0

    def __post_init__(self):
        # declare ROS params
        self._declare_ros_params(namespace="velocity_planner")


@dataclass
class RPInterfaceParams(BaseParams):
    """
    Class for reactive planner interface parameters.

    :var rp_config_yaml: path to reactive planner yaml configuration
    :var d_min:
    :var d_max:
    :var t_min:
    """
    # path to folder with reactive planner yaml configuration
    rp_config_yaml: str = \
        "../../../../src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/param/edgar_rp_config.yaml"

    # sampling settings
    d_min: float = -3.0
    d_max: float = 3.0
    t_min: float = 0.4

    def __post_init__(self):
        """Initialize ROS params."""
        # declare ROS params
        self._declare_ros_params(namespace="rp_interface")

        # get absolute path from default configs
        self.path_rp_config = get_absolute_path_from_package(relative_path=self.rp_config_yaml,
                                                             package_name="cr2autoware")

    def get_ros_param(self, param_name: str) -> Any:
        """
        Get ROS parameter value.

        :param param_name: name of ROS parameter
        """
        namespace = "rp_interface"
        ros_param_name = namespace + "." + param_name
        ros_param_value = self._node.get_parameter(ros_param_name).get_parameter_value()

        dtype = type(self.__getitem__(param_name))
        if dtype == float:
            return ros_param_value.double_value
        elif dtype == int:
            return ros_param_value.integer_value
        elif dtype == str:
            return ros_param_value.string_value
        elif dtype == bool:
            return ros_param_value.string_value
        else:
            raise TypeError(f"<get_ros_params(): Invalid Type {dtype} for ROS Parameter")


@dataclass
class TrajectoryPlannerParams(BaseParams):
    """
    Base Class for trajectory planner parameters.

    :var trajectory_planner_type: type of trajectory planner to use (1: reactive planner)
    :var planner_update_time: re-planning time for cyclic re-planning (in seconds)
    :var planning_horizon: planning horizon (in seconds)
    :var safety_margin: safety buffer to obstacles (in meters)
    """
    # type of trajectory planner to use (1: reactive planner)
    trajectory_planner_type: int = 1

    # re-planning time for cyclic re-planning (in seconds)
    planner_update_time: float = 0.5
    # planning horizon (in seconds)
    planning_horizon: float = 5.0
    # safety buffer to obstacles (in meters)
    safety_margin: float = 0.8

    def __post_init__(self):
        """Initialize ROS params."""
        # declare ROS params
        self._declare_ros_params(namespace="trajectory_planner")


@dataclass
class CR2AutowareParams:
    """
    Main parameter class for the CR2Auto node.

    :var _node: reference to ROS node
    :var general: GeneralParams
    :var scenario: ScenarioParams
    :var vehicle: VehicleParams
    :var velocity_planner: VelocityPlannerParams
    :var trajectory_planner: TrajectoryPlannerParams
    :var rp_interface: RPInterfaceParams
    """
    # reference to ROS node
    _node: "Cr2Auto"

    # set fields for subclasses
    general: GeneralParams = field(init=False)
    scenario: ScenarioParams = field(init=False)
    vehicle: VehicleParams = field(init=False)
    velocity_planner: VelocityPlannerParams = field(init=False)
    trajectory_planner: TrajectoryPlannerParams = field(init=False)
    rp_interface: RPInterfaceParams = field(init=False)

    def __post_init__(self):
        """Initialize subclasses and pass reference to node."""
        # initialize subclasses and pass reference to node
        self.general = GeneralParams(_node=self._node)
        self.scenario = ScenarioParams(_node=self._node)
        self.vehicle = VehicleParams(_node=self._node)
        self.velocity_planner = VelocityPlannerParams(_node=self._node)
        self.trajectory_planner = TrajectoryPlannerParams(_node=self._node)
        self.rp_interface = RPInterfaceParams(_node=self._node)
