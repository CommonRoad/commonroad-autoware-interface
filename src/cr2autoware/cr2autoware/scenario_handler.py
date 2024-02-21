# standard imports
from decimal import Decimal
import glob
import math
import os
import typing
from typing import Any
from typing import Dict
from typing import List
from typing import Union

# third party
import numpy as np
from pyproj import Proj
import utm
import yaml

# ROS msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterValue
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.publisher import Publisher
from std_msgs.msg import Header

# Autoware msgs
from autoware_auto_perception_msgs.msg import DetectedObjects  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects  # type: ignore
from autoware_auto_planning_msgs.msg import TrajectoryPoint  # type: ignore
from dummy_perception_publisher.msg import Object  # type: ignore

# commonroad-io imports
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.obstacle import StaticObstacle
from commonroad.scenario.scenario import Scenario as CRScenario
from commonroad.scenario.state import InitialState
from commonroad.scenario.state import TraceState
from commonroad.scenario.lanelet import LaneletNetwork

# commonroad-dc imports
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle

# crdesigner imports
from crdesigner.common.config.lanelet2_config import lanelet2_config
from crdesigner.common.config.general_config import general_config
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

# cr2autowar
import cr2autoware.utils as utils


# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto

# TODO: Figure out how to load from CR xml file


class ScenarioHandler:
    """
    Handles communication with Autoware for CommonRoad Scenario relevant data.
    Keeps an up to date state of the current scenario in CommonRoad format.

    ======== Publishers:
    _OBSTACLE_PUBLISHER: "/simulation/dummy_perception_publisher/object_info"

    ======== Subscribers:
    "/perception/object_recognition/detection/objects" (subscribes to DetectedObjects messages)
    "/perception/object_recognition/objects" (subscribes to PredictedObjects messages)
    """

    # Constants and parameters
    MAP_PATH: str
    LEFT_DRIVING: bool
    ADJACENCIES: bool
    VERBOSE: bool = False

    _logger: RcutilsLogger

    _scenario: CRScenario
    _origin_transformation: List[float]
    _node: "Cr2Auto"
    _last_msg: Dict[str, Any] = {}
    _dynamic_obstacles_map: Dict[int, int] = {}
    # We update obstacles in the scenario with every msg
    # to publish the initial obstacles, they need to be stored
    _initial_obstacles: List[Union[DynamicObstacle, StaticObstacle]] = []

    # Publishers
    _OBSTACLE_PUBLISHER: Publisher

    def __init__(self, node: "Cr2Auto"):
        self._node = node
        self._logger = node.get_logger().get_child("scenario_handler")

        # Get parameters from the node
        self._init_parameters()

        # Loading the map_config.yaml file
        map_config = self._read_map_config(self.MAP_PATH)
        projection_string = self._get_projection_string(map_config)

        # Loading the map from file
        dt = self._node.get_parameter("scenario.dt").get_parameter_value().double_value
        self._scenario = self._load_initial_scenario(
            self.MAP_PATH, projection_string, self.LEFT_DRIVING, self.ADJACENCIES, dt=dt
        )
        self._road_boundary = self._create_initial_road_boundary()
        self._origin_transformation = self._get_origin_transformation(
            map_config, Proj(projection_string), self._scenario
        )

        # Subscribing to relevant topics
        self._init_subscriptions(self._node)

        # Creating publishers
        self._init_publishers(self._node)

        # Initialize list of current z values
        self._z_list = [None, None]  # [initial_pose_z, goal_pose_z]
        self._initialpose3d_z = 0.0  # z value of initialpose3d

    def _init_parameters(self) -> None:
        def _get_parameter(name: str) -> ParameterValue:
            return self._node.get_parameter(name).get_parameter_value()

        self.MAP_PATH = _get_parameter("general.map_path").string_value
        if not os.path.exists(self.MAP_PATH):
            raise ValueError("Can't find given map path: %s" % self.MAP_PATH)

        self.LEFT_DRIVING = _get_parameter("scenario.left_driving").bool_value
        self.ADJACENCIES = _get_parameter("scenario.adjacencies").bool_value
        self.VERBOSE = _get_parameter("general.detailed_log").bool_value
        if self.VERBOSE:
            from rclpy.logging import LoggingSeverity

            self._logger.set_level(LoggingSeverity.DEBUG)

    def _read_map_config(self, map_path: str) -> Dict[str, Any]:
        map_config = {}

        map_config_tmp = list(glob.iglob(os.path.join(map_path, ("map_config" + "*.[yY][aA][mM][lL]"))))
        if not map_config_tmp:
            raise ValueError("Couldn't load map origin! No YAML file exists in: %s" % map_path)

        with open(map_config_tmp[0]) as f:
            map_config_yaml = yaml.load(f, Loader=yaml.SafeLoader)

        ros_parameters = map_config_yaml["/**"]["ros__parameters"]

        # Read the map origin
        map_config["aw_origin_latitude"] = Decimal(ros_parameters["map_origin"]["latitude"])
        map_config["aw_origin_longitude"] = Decimal(ros_parameters["map_origin"]["longitude"])

        # Verify the origin
        if (
            abs(map_config["aw_origin_longitude"]) > 180
            or abs(map_config["aw_origin_latitude"]) > 90
        ):
            # In some CR scenarios, the lat/long values are set to non-existing values. Use 0 values instead
            self._logger.warn(
                f"Invalid lat/lon coordinates: lat {map_config['aw_origin_latitude']}, "
                f"lon {map_config['aw_origin_longitude']}. "
                "Using lat 0 long 0 instead."
            )
            map_config["aw_origin_latitude"] = 0
            map_config["aw_origin_longitude"] = 0

        # Read the use_local_coordinates parameter
        try:
            map_config["use_local_coordinates"] = bool(ros_parameters["use_local_coordinates"])
        except KeyError:
            map_config["use_local_coordinates"] = False
        finally:
            if map_config["use_local_coordinates"]:
                self._logger.info("Using same coordinates in CommonRoad and Autoware.")
            else:
                self._logger.info("Converting coordinates from CommonRoad to Autoware.")

        return map_config

    def _get_projection_string(self, map_config: Dict[str, Any]) -> str:
        aw_origin_latitude = map_config["aw_origin_latitude"]
        aw_origin_longitude = map_config["aw_origin_longitude"]
        utm_str = utm.from_latlon(float(aw_origin_latitude), float(aw_origin_longitude))
        projection_string = "+proj=utm +zone=%d +datum=WGS84 +ellps=WGS84" % (utm_str[2])
        self._logger.debug(f"Proj string: {projection_string}")
        return projection_string

    def _get_origin_transformation(
        self, map_config: Dict[str, Any], projection: Proj, scenario: CRScenario
    ) -> List[Union[float, Any]]:
        aw_origin_x, aw_origin_y = projection(
            map_config["aw_origin_longitude"], map_config["aw_origin_latitude"]
        )

        self._logger.debug(
            "Autoware origin lat: %s,   origin lon: %s"
            % (map_config["aw_origin_latitude"], map_config["aw_origin_longitude"])
        )
        self._logger.debug("Autoware origin x: %s,   origin y: %s" % (aw_origin_x, aw_origin_y))

        # Get CommonRoad map origin
        cr_origin_lat = Decimal(scenario.location.gps_latitude)
        cr_origin_lon = Decimal(scenario.location.gps_longitude)
        cr_origin_x, cr_origin_y = projection(cr_origin_lon, cr_origin_lat)

        self._logger.debug(
            "CommonRoad origin lat: %s,   origin lon: %s" % (cr_origin_lat, cr_origin_lon)
        )
        self._logger.debug("CommonRoad origin x: %s,   origin y: %s" % (cr_origin_x, cr_origin_y))

        # using local CR scenario coordinates => same origin
        if map_config["use_local_coordinates"]:
            origin_transformation = [0.0, 0.0]
        else:
            origin_transformation = [aw_origin_x - cr_origin_x, aw_origin_y - cr_origin_y]
        self._logger.info(
            f"origin transformation x: {origin_transformation[0]}, "
            f"origin transformation y: {origin_transformation[1]}"
        )
        return origin_transformation

    def _load_initial_scenario(
        self,
        map_path: str,
        projection_string: str,
        left_driving: bool,
        adjacencies: bool,
        dt: float,
    ) -> CRScenario:
        """Initialize the scenario either from a CommonRoad scenario file or from Autoware map.

        Transforms Autoware map to CommonRoad scenario if no CommonRoad scenario file is found
        in the `map_path` directory.
        """
        # find open street map file in folder map_path
        map_filename = list(glob.iglob(os.path.join(map_path, "*.[oO][sS][mM]")))[0]
        map_filename_cr = list(glob.iglob(os.path.join(map_path, "*.[xX][mM][lL]")))

        # if no commonroad file is present in the directory then create it
        if map_filename_cr is not None and len(map_filename_cr) > 0:
            scenario = self._build_scenario_from_commonroad(map_filename_cr[0])
        else:
            scenario = self._build_scenario_from_autoware(
                map_filename, projection_string, left_driving, adjacencies, dt=dt
            )
        scenario.convert_to_2d()
        self._initial_obstacles.extend(scenario.static_obstacles)
        self._initial_obstacles.extend(scenario.dynamic_obstacles)
        return scenario

    def _create_initial_road_boundary(self) -> pycrcc.CollisionObject:
        """
        Creates road boundary collision obstacles for the lanelet network of the loaded scenario.
        Currently, we assume that the lanelet network of the initial map is fixed.
        """
        if self._scenario is None:
            self._logger.error("ScenarioHandler._create_initial_road_boundary(): No CR scenario given.")

        # TODO: make method and params configurable
        road_boundary_collision_object = create_road_boundary_obstacle(self._scenario,
                                                                       method='obb_rectangles',
                                                                       width=2e-3,
                                                                       return_scenario_obstacle=False)

        return road_boundary_collision_object

    def _build_scenario_from_commonroad(self, map_filename_cr: str) -> CRScenario:
        self._logger.info(
            "Found a CommonRoad scenario file inside the directory. "
            "Loading map from CommonRoad scenario file"
        )
        commonroad_reader = CommonRoadFileReader(map_filename_cr)
        return commonroad_reader.open()[0]

    def _build_scenario_from_autoware(
        self,
        map_filename: str,
        projection_string: str,
        left_driving: bool,
        adjacencies: bool,
        dt: float,
    ) -> CRScenario:
        self._logger.info(
            "Could not find a CommonRoad scenario file inside the directory. "
            "Creating from autoware map via Lanelet2CommonRoad conversion instead"
        )

        general_config.proj_string_cr = projection_string
        lanelet2_config.left_driving = left_driving
        lanelet2_config.adjacencies = adjacencies
        lanelet2_config.translate = True

        scenario = lanelet_to_commonroad(input_file=map_filename, general_conf=general_config,
                                         lanelet2_conf=lanelet2_config)
        
        scenario.dt = dt
        return scenario

    def _init_subscriptions(self, node: "Cr2Auto") -> None:
        # Callbacks to save the latest messages for later processing
        node.create_subscription(
            DetectedObjects,
            "/perception/object_recognition/detection/objects",
            lambda msg: self._last_msg.update({"static_obstacle": msg}),
            1,
            callback_group=node.callback_group,
        )
        node.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            lambda msg: self._last_msg.update({"dynamic_obstacle": msg}),
            1,
            callback_group=node.callback_group,
        )
        node.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose3d",
            lambda msg: self._last_msg.update({"initial_pose": msg}),
            1,
            callback_group=node.callback_group,
        )
        node.create_subscription(
            PoseStamped,
            "/planning/mission_planning/echo_back_goal_pose",
            lambda msg: self._last_msg.update({"goal_pose": msg}),
            1,
            callback_group=node.callback_group,
        )

    def _init_publishers(self, node: "Cr2Auto") -> None:
        self._OBSTACLE_PUBLISHER = node.create_publisher(
            Object,
            "/simulation/dummy_perception_publisher/object_info",
            1,
        )

    @property
    def scenario(self) -> CRScenario:
        return self._scenario

    @property
    def lanelet_network(self) -> LaneletNetwork:
        return self._scenario.lanelet_network

    @property
    def road_boundary(self) -> pycrcc.CollisionObject:
        return self._road_boundary

    @property
    def origin_transformation(self) -> List[Union[float, Any]]:
        return self._origin_transformation

    def update_scenario(self):
        """Trigger an update of the scenario.

        This function can be called to update the scenario with the latest messages.
        Typically it is called by a timer callback.
        """
        self._logger.info("Updating scenario")
        self._process_static_obs()
        self._process_dynamic_obs()

    def _process_static_obs(self) -> None:
        last_message = self._last_msg.get("static_obstacle")  # type: DetectedObjects
        if last_message is None:
            return
        # TODO: remove the dynamic obstacles from the static list
        temp_pose = PoseStamped()
        temp_pose.header = last_message.header

        # remove all static obstacles from the scenario
        # Bug in CR scenario where List[StaticObstacle] is not allowed - fixed in newest version
        self._scenario.remove_obstacle(self._scenario.static_obstacles)  # type: ignore

        # add all static obstacles from the last message to the scenario
        for box in last_message.objects:
            # TODO: CommonRoad can also consider uncertain states of obstacles, which we could derive from the covariances
            temp_pose.pose.position.x = box.kinematics.pose_with_covariance.pose.position.x
            temp_pose.pose.position.y = box.kinematics.pose_with_covariance.pose.position.y
            temp_pose.pose.position.z = box.kinematics.pose_with_covariance.pose.position.z
            temp_pose.pose.orientation.x = box.kinematics.pose_with_covariance.pose.orientation.x
            temp_pose.pose.orientation.y = box.kinematics.pose_with_covariance.pose.orientation.y
            temp_pose.pose.orientation.z = box.kinematics.pose_with_covariance.pose.orientation.z
            temp_pose.pose.orientation.w = box.kinematics.pose_with_covariance.pose.orientation.w
            pose_map = self._node.transform_pose(temp_pose, "map")
            if pose_map is None:
                continue

            pos = utils.map2utm(self.origin_transformation, pose_map.pose.position)
            orientation = utils.quaternion2orientation(pose_map.pose.orientation)
            width = box.shape.dimensions.y
            length = box.shape.dimensions.x

            obs_state = InitialState(
                position=pos,
                orientation=orientation,
                velocity=0,
                acceleration=0,
                yaw_rate=0,
                slip_angle=0,
                time_step=0,
            )

            obs_id = self._scenario.generate_object_id()
            # TODO: get object type from autoware --> see https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/ObjectClassification.idl
            obs_type = ObstacleType.UNKNOWN
            obs_shape = Rectangle(width=width, length=length)

            self._scenario.add_objects(StaticObstacle(obs_id, obs_type, obs_shape, obs_state))

    # TODO: This function is too complicated -> split it up
    def _process_dynamic_obs(self) -> None:
        """Convert dynamic autoware obstacles to commonroad obstacles and add them to the scenario."""
        last_message = self._last_msg.get("dynamic_obstacle")  # type: PredictedObjects
        if last_message is None:
            return

        for obstacle in last_message.objects:
            position = utils.map2utm(
                self._origin_transformation,
                obstacle.kinematics.initial_pose_with_covariance.pose.position,
            )
            orientation = utils.quaternion2orientation(
                obstacle.kinematics.initial_pose_with_covariance.pose.orientation
            )
            velocity = obstacle.kinematics.initial_twist_with_covariance.twist.linear.x
            yaw_rate = obstacle.kinematics.initial_twist_with_covariance.twist.angular.z
            width = obstacle.shape.dimensions.y
            length = obstacle.shape.dimensions.x
            time_step = 0
            traj = []
            highest_conf_val = 0
            highest_conf_idx = 0
            for i in range(len(obstacle.kinematics.predicted_paths)):
                conf_val = obstacle.kinematics.predicted_paths[i].confidence
                if conf_val > highest_conf_val:
                    highest_conf_val = conf_val
                    highest_conf_idx = i

            obj_traj_dt = obstacle.kinematics.predicted_paths[highest_conf_idx].time_step.nanosec

            if obj_traj_dt > self.scenario.dt * 1e9:
                # upsample predicted path of obstacles to match dt
                if obj_traj_dt % (self.scenario.dt * 1e9) == 0.0:
                    dt_ratio = int(obj_traj_dt / (self.scenario.dt * 1e9)) + 1
                else:
                    dt_ratio = math.ceil(obj_traj_dt / (self.scenario.dt * 1e9))

                for point in obstacle.kinematics.predicted_paths[highest_conf_idx].path:
                    traj.append(point)
                    if len(traj) >= 2:
                        utils.upsample_trajectory(traj, dt_ratio)

            elif obj_traj_dt < self.scenario.dt * 1e9:
                # downsample predicted path of obstacles to match dt.
                # if the time steps are divisible without reminder,
                # get the trajectories at the steps according to ratio
                if (self.scenario.dt * 1e9) % obj_traj_dt == 0.0:
                    dt_ratio = (self.scenario.dt * 1e9) / obj_traj_dt
                    for idx, point in enumerate(
                        obstacle.kinematics.predicted_paths[highest_conf_idx].path
                    ):
                        if (idx + 1) % dt_ratio == 0:
                            traj.append(point)
                else:
                    # make interpolation according to time steps
                    dt_ratio = math.ceil((self.scenario.dt * 1e9) / obj_traj_dt)
                    for idx, point in enumerate(
                        obstacle.kinematics.predicted_paths[highest_conf_idx].path
                    ):
                        if (idx + 1) % dt_ratio == 0:
                            point_1 = obstacle.kinematics.predicted_paths[highest_conf_idx].path[
                                idx - 1
                            ]
                            point_2 = point
                            new_point = utils.traj_linear_interpolate(
                                point_1, point_2, obj_traj_dt, self.scenario.dt * 1e9
                            )
                            traj.append(new_point)
            else:
                for point in obstacle.kinematics.predicted_paths[highest_conf_idx].path:
                    traj.append(point)

            # TODO: Remove nested list when code is testable.
            object_id_aw: int = obstacle.object_id.uuid
            aw_id_list = [list(value) for value in self._dynamic_obstacles_map.values()]
            if list(object_id_aw) not in aw_id_list:
                dynamic_obstacle_initial_state = InitialState(
                    position=position,
                    orientation=orientation,
                    velocity=velocity,
                    acceleration=0.0,
                    yaw_rate=yaw_rate,
                    slip_angle=0.0,
                    time_step=time_step,
                )
                object_id_cr = self._scenario.generate_object_id()
                self._dynamic_obstacles_map[object_id_cr] = object_id_aw
                self._add_dynamic_obstacle(
                    dynamic_obstacle_initial_state, traj, width, length, object_id_cr, time_step
                )
            else:
                for key, value in self._dynamic_obstacles_map.items():
                    if np.array_equal(object_id_aw, value):
                        dynamic_obs = self._scenario.obstacle_by_id(key)
                        if dynamic_obs:
                            dynamic_obs.initial_state = InitialState(
                                position=position,
                                orientation=orientation,
                                velocity=velocity,
                                acceleration=0.0,
                                yaw_rate=yaw_rate,
                                slip_angle=0.0,
                                time_step=time_step,
                            )
                            dynamic_obs.obstacle_shape = Rectangle(width=width, length=length)
                            if len(traj) > 2:
                                dynamic_obs.prediction = TrajectoryPrediction(
                                    self._node._awtrajectory_to_crtrajectory(
                                        2, dynamic_obs.initial_state.time_step, traj
                                    ),
                                    dynamic_obs.obstacle_shape,
                                )

    def _add_dynamic_obstacle(
        self,
        initial_state: TraceState,
        traj: List[TrajectoryPoint],
        width: float,
        length: float,
        object_id: int,
        time_step: int,
    ) -> None:
        """Add dynamic obstacles with their trajectory.

        :param initial_state: initial state of obstacle
        :param traj: trajectory of obstacle
        :param width: width of obstacle
        :param length: length of obstacle
        :param object_id: id of obstacle
        :param time_step: time step of the obstacle
        """
        dynamic_obstacle_shape = Rectangle(width=width, length=length)
        # ToDo: get object type from autoware
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle_id = object_id
        if len(traj) > 2:
            # create the trajectory of the obstacle, starting at time_step
            dynamic_obstacle_trajectory = self._node._awtrajectory_to_crtrajectory(
                2, time_step, traj
            )

            # create the prediction using the trajectory and the shape of the obstacle
            dynamic_obstacle_prediction = TrajectoryPrediction(
                dynamic_obstacle_trajectory, dynamic_obstacle_shape
            )

            dynamic_obstacle = DynamicObstacle(
                dynamic_obstacle_id,
                dynamic_obstacle_type,
                dynamic_obstacle_shape,
                initial_state,
                dynamic_obstacle_prediction,
            )
        else:
            dynamic_obstacle = DynamicObstacle(
                dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape, initial_state
            )
        # add dynamic obstacle to the scenario
        self.scenario.add_objects(dynamic_obstacle)

    def publish_initial_obstacles(self):
        """Publish initial static and dynamic obstacles from CR in form of AW 2D Dummy cars."""
        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()
        header.frame_id = "map"

        for obstacle in self._initial_obstacles:
            object_msg = utils.create_object_base_msg(header, self.origin_transformation, obstacle)
            if isinstance(obstacle, DynamicObstacle):
                try:
                    # Bug in CommonRoad?: initial_state is not of type InitialState (warumauchimmer)
                    state: InitialState = obstacle.initial_state  # type: ignore
                    object_msg.initial_state.twist_covariance.twist.linear.x = state.velocity
                    object_msg.initial_state.accel_covariance.accel.linear.x = state.acceleration
                except AttributeError as e:
                    self._logger.error(
                        "Error during publish_initial_obstacles. \n"
                        f"Obstacle {obstacle} has no velocity or acceleration."
                    )
                    self._logger.debug(e)
                object_msg.max_velocity = 20.0
                object_msg.min_velocity = -10.0
            self._OBSTACLE_PUBLISHER.publish(object_msg)
            self._logger.debug(utils.log_obstacle(object_msg, isinstance(obstacle, StaticObstacle)))

    def get_z_coordinate(self):
        """Calculate the mean of the z coordinate from initial_pose and goal_pose."""
        new_initial_pose_z = None
        new_goal_pose_z = None
        goal_pose_z = self._z_list[1]

        new_initial_pose = self._last_msg.get("initial_pose")
        new_goal_pose = self._last_msg.get("goal_pose")   
 
        # only consider values if z is not None or 0.0
        if new_initial_pose is not None and new_initial_pose.pose.pose.position.z != 0.0:       
            new_initial_pose_z = new_initial_pose.pose.pose.position.z

        if new_goal_pose is not None and new_goal_pose.pose.position.z != 0.0:
            new_goal_pose_z = new_goal_pose.pose.position.z

        # check if new goal_pose or initial_pose is published
        if new_goal_pose_z != goal_pose_z:
            if new_initial_pose_z != self._initialpose3d_z and new_initial_pose_z is not None:
                # both initial_pose and goal_pose changed
                self._initialpose3d_z = new_initial_pose_z
                self._z_list[0] = new_initial_pose_z
                self._z_list[1] = new_goal_pose_z
            elif goal_pose_z is None:
                # goal_pose initially None and now changed (first GoalPose got published)
                self._z_list[1] = new_goal_pose_z
            else:
                # only goal_pose changed and goal_pose was not None before (new GoalPose got published)
                self._z_list[0] = goal_pose_z
                self._z_list[1] = new_goal_pose_z
        elif new_initial_pose_z != self._initialpose3d_z and new_initial_pose_z is not None:
            # only initial_pose changed; set goal_pose to None
            self._initialpose3d_z = new_initial_pose_z
            self._z_list[0] = new_initial_pose_z
            self._z_list[1] = None

        valid_z_list = [i for i in [self._z_list[0], self._z_list[1]] if i is not None]
        if not valid_z_list:
            z = 0.0
            self._logger.info("Z is either not found or is 0.0")
            return z
        z = np.median(valid_z_list)
        return z