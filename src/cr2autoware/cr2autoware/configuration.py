"""
This module defines the parameter structure for the CR2Auto node.
All ROS parameters are encapsulated within one dataclass CR2AutowareParams (with different sub-dataclasses) and are
declared and set upon initialization of the node.
"""

# typing
import typing
from typing import Any, List

# standard imports
from dataclasses import dataclass, field, fields
import yaml

# ROS imports
from rclpy.parmaeter import Parameter

# cr2autoware imports
from cr2autoware.utils import get_absolute_path_from_package

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


@dataclass
class BaseParams:
    """
    CR2Auto base parameters.
    """
    # reference to CR2Auto ROS node
    _node: "Cr2Auto"

    def __getitem__(self, item: str) -> Any:
        """Getter for base parameter value."""
        try:
            value = self.__getattribute__(item)
        except AttributeError as e:
            raise KeyError(f"{item} is not a parameter of {self.__class__.__name__}") from e
        return value

    def __setitem__(self, key: str, value: Any):
        """Setter for item."""
        try:
            self.__setattr__(key, value)
        except AttributeError as e:
            raise KeyError(f"{key} is not a parameter of {self.__class__.__name__}") from e

    def _declare_ros_params(self, namespace: str):
        """Function to declare ROS params in the associated ROS2 node (self._node) """
        for field_info in fields(self):
            # ignore _node field
            if not field_info.name.startswith('_'):
                key = field_info.name
                val = getattr(self, key)
                ros_param_name = namespace + "." + key
                self._node.declare_parameter(ros_param_name, val).value


@dataclass
class GeneralParams(BaseParams):
    """Class for general parameters"""
    # path to map directory (Launch arg)
    map_path: str = ""
    # path to CR solution file (Launch arg)
    solution_file: str = ""

    # use spot prediction
    enable_spot: bool = False

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
        # declare ROS params
        self._declare_ros_params(namespace="scenario")


@dataclass
class VehicleParams(BaseParams):
    """
    Class for handling vehicle parameters
    - Static vehicle parameters are read from the vehicle_info.param.yaml file in the vehicle_description ROS package
    - Additional static (e.g., rear axle distance) and dynamic (e.g., max-velocity) parameters are defined here and
      are declared as ROS parameters for the CR2Auto node
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
    """Class for velocity planner parameters"""
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
    """Class for reactive planner interface parameters"""
    # path to folder with default yaml configurations (TODO: deprecated, remove with new version of reactive planner)
    default_yaml_folder: str = \
        "../../../../src/universe/autoware.universe/planning/tum_commonroad_planning/reactive-planner/configurations/defaults/"

    # sampling settings
    d_min: float = -3.0
    d_max: float = 3.0
    t_min: float = 0.4

    def __post_init__(self):
        # declare ROS params
        self._declare_ros_params(namespace="rp_interface")

        # get absolute path from default configs
        self.dir_config_default = get_absolute_path_from_package(relative_path=self.default_yaml_folder,
                                                                 package_name="cr2autoware")

    def get_ros_param(self, param_name: str):
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
    """Base Class for trajectory planner parameters"""
    # type of trajectory planner to use (1: reactive planner)
    trajectory_planner_type: int = 1

    # re-planning time for cyclic re-planning (in seconds)
    planner_update_time: float = 0.5
    # planning horizon (in seconds)
    planning_horizon: float = 5.0

    def __post_init__(self):
        # declare ROS params
        self._declare_ros_params(namespace="trajectory_planner")


@dataclass
class CR2AutowareParams:
    """
    Main parameter class for the CR2Auto node
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
        # initialize subclasses and pass reference to node
        self.general = GeneralParams(_node=self._node)
        self.scenario = ScenarioParams(_node=self._node)
        self.vehicle = VehicleParams(_node=self._node)
        self.velocity_planner = VelocityPlannerParams(_node=self._node)
        self.trajectory_planner = TrajectoryPlannerParams(_node=self._node)
        self.rp_interface = RPInterfaceParams(_node=self._node)
