"""
This module defines the configuration structure for the CR2Auto node.
All ROS parameters are encapsulated within one dataclass CR2AutowareParams (with different sub-dataclasses) and are
declared and set upon initialization of the node.
"""

# typing
from typing import Any, Type

# standard imports
from dataclasses import dataclass, field

# cr2autoware imports
from cr2autoware.utils import get_absolute_path_from_package


@dataclass
class BaseParams:
    """
    CR2Auto base parameters.
    """

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


@dataclass
class VehicleParams(BaseParams):
    """Class for vehicle parameters"""
    # center of gravity to axle distances (in meter)
    cg_to_front: float = 1.0
    cg_to_rear: float = 1.0
    # overhang lengths (in meter)
    front_overhang: float = 0.5
    rear_overhang: float = 0.5
    # width (in meter)
    width: float = 2.0
    # velocity limits (in m/s)
    max_velocity: float = 10.0
    min_velocity: float = 0.0


@dataclass
class VelocityPlannerParams(BaseParams):
    """Class for velocity planner parameters"""
    # lookahead distance (in meters)
    lookahead_dist: float = 3.0
    # lookahead time (in seconds)
    lookahead_time: float = 0.8
    # initial velocity (in m/s) (TODO: not yet implemented -> remove?; should only be used for lanelet2 maps)
    init_velocity: float = 5.0


@dataclass
class RPInterfaceParams(BaseParams):
    """Class for reactive planner interface parameters"""
    # path to folder with default yaml configurations (TODO: deprecated, remove with new version)
    default_yaml_folder: str = \
        "../../../../src/universe/autoware.universe/planning/tum_commonroad_planning/reactive-planner/configurations/defaults/"

    # sampling settings
    d_min: float = -3.0
    d_max: float = 3.0
    t_min: float = 0.4

    def __post_init__(self):
        """get absolute path for default configs"""
        self.dir_config_default = get_absolute_path_from_package(relative_path=self.default_yaml_folder,
                                                                 package_name="cr2autoware")


@dataclass
class TrajectoryPlannerParams(BaseParams):
    """Base Class for trajectory planner parameters"""
    # type of trajectory planner to use (1: reactive planner)
    trajectory_planner_type: int = 1

    # re-planning time for cyclic re-planning (in seconds)
    planner_update_time: float = 0.5
    # planning horizon (in seconds)
    planning_horizon: float = 5.0


@dataclass
class CR2AutowareParams(BaseParams):
    """
    Main parameter class for the CR2Auto node
    """

    general: GeneralParams = field(default_factory=GeneralParams)
    scenario: ScenarioParams = field(default_factory=ScenarioParams)
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    velocity_planner: VelocityPlannerParams = field(default_factory=VelocityPlannerParams)
    trajectory_planner: TrajectoryPlannerParams = field(default_factory=TrajectoryPlannerParams)

    def __post__init__(self):
        """add field for specific trajectory planner parameters depending on specified type"""
        planner_type = self.trajectory_planner.trajectory_planner_type
        if planner_type == 1:
            self.rp_interface: RPInterfaceParams = field(default_factory=RPInterfaceParams)
        else:
            raise ValueError(f"Given trajectory planner type: {planner_type} is invalid")




#====================================================
# TODO: Old Remove after testing new config structure!!
class TrajectoryPlannerBaseParams:
    """Shared parameters for trajectory planner."""

    def __init__(self, get_parameter):
        self.get_parameter = get_parameter
        self._create_ego_vehicle_info()

    def _create_ego_vehicle_info(self):
        """Compute the dimensions of the ego vehicle."""
        cg_to_front = self.get_parameter("vehicle.cg_to_front").get_parameter_value().double_value
        cg_to_rear = self.get_parameter("vehicle.cg_to_rear").get_parameter_value().double_value
        width = self.get_parameter("vehicle.width").get_parameter_value().double_value
        front_overhang = (
            self.get_parameter("vehicle.front_overhang").get_parameter_value().double_value
        )
        rear_overhang = (
            self.get_parameter("vehicle.rear_overhang").get_parameter_value().double_value
        )

        self.vehicle_length = front_overhang + cg_to_front + cg_to_rear + rear_overhang
        self.vehicle_width = width
        self.vehicle_wheelbase = cg_to_front + cg_to_rear


class RPParams(TrajectoryPlannerBaseParams):
    """Parameters for initializing reactive planner."""

    def __init__(self, get_parameter):
        super().__init__(get_parameter)
        self.d_min = (
            self.get_parameter("reactive_planner.sampling.d_min")
            .get_parameter_value()
            .integer_value
        )
        self.d_max = (
            self.get_parameter("reactive_planner.sampling.d_max")
            .get_parameter_value()
            .integer_value
        )
        self.t_min = (
            self.get_parameter("reactive_planner.sampling.t_min").get_parameter_value().double_value
        )
        self.planning_horizon = (
            self.get_parameter("reactive_planner.planning.planning_horizon")
            .get_parameter_value()
            .double_value
        )
        self.dir_config_default = get_absolute_path_from_package(
            self.get_parameter("reactive_planner.default_yaml_folder")
            .get_parameter_value()
            .string_value,
            package_name="cr2autoware",
        )
