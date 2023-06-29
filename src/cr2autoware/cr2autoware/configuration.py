from cr2autoware.utils import get_absolute_path_from_package


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
