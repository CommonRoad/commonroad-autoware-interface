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
from typing import Optional

# third party
import numpy as np
from pyproj import Proj
import utm
import yaml

# ROS msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Polygon as PolygonMsg
from rclpy.publisher import Publisher
from std_msgs.msg import Header

# Autoware msgs
from autoware_auto_perception_msgs.msg import PredictedObjects  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObject  # type: ignore
from autoware_auto_perception_msgs.msg import ObjectClassification  # type: ignore

# commonroad-io imports
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import StaticObstacle
from commonroad.scenario.scenario import Scenario as CRScenario
from commonroad.scenario.state import InitialState
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.scenario.lanelet import LaneletNetwork

# commonroad-dc imports
import commonroad_dc.pycrcc as pycrcc
from commonroad_dc.boundary.boundary import create_road_boundary_obstacle

# crdesigner imports
from crdesigner.common.config.general_config import general_config
from crdesigner.common.config.lanelet2_config import lanelet2_config
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

# cr2autoware
from .base import BaseHandler
from ..common.utils.type_mapping import aw_to_cr_shape
from ..common.utils.type_mapping import aw_to_cr_shape_updater
from ..common.utils.type_mapping import get_classification_with_highest_probability
from ..common.utils.type_mapping import aw_to_cr_obstacle_type
from ..common.utils.transform import quaternion2orientation
from ..common.utils.transform import map2utm
from ..common.utils.geometry import upsample_trajectory
from ..common.utils.geometry import traj_linear_interpolate
from ..common.ros_interface.create import create_subscription

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto

# subscriber specifications
from ..common.ros_interface.specs_subscriptions import spec_objects_sub


class ScenarioHandler(BaseHandler):
    """
    Handles communication with Autoware for CommonRoad Scenario relevant data.
    Keeps an up to date state of the current scenario in CommonRoad format.

    ======== Publishers:

    ======== Subscribers:
    "/perception/object_recognition/objects" (subscribes to PredictedObjects messages)
    """

    # Constants and parameters
    MAP_PATH: str
    LEFT_DRIVING: bool
    ADJACENCIES: bool

    _scenario: CRScenario
    _dt: float
    _origin_transformation: List[float]
    _last_msg: Dict[str, Any] = {}
    _dynamic_obstacles_map: Dict[int, int] = {}

    def __init__(self, node: "Cr2Auto"):
        # init base class
        super().__init__(node=node,
                         logger=node.get_logger().get_child("scenario_handler"),
                         verbose=node.verbose)

        # Get parameters from the node
        self._init_parameters()

        # initialize subscriptions to relevant topics
        self._init_subscriptions()

        # initialize publishers
        self._init_publishers()

        # Loading the map_config.yaml file
        map_config = self._read_map_config(self.MAP_PATH)
        projection_string = self._get_projection_string(map_config)

        # Loading the map from file
        self._scenario = self._load_initial_scenario(
            self.MAP_PATH, projection_string, self.LEFT_DRIVING, self.ADJACENCIES, dt=self._dt
        )
        # Create static road boundary
        self._road_boundary = self._create_initial_road_boundary()
        # create origin transformation
        self._origin_transformation = self._get_origin_transformation(
            map_config, Proj(projection_string), self._scenario)

        # Initialize params for computing elevation (z-coordinate)
        self._z_list = [None, None]  # [initial_pose_z, goal_pose_z]
        self._initialpose3d_z = 0.0  # z value of initialpose3d
        self._z_coordinate = 0.0     # current elevation, i.e., z-coordinate

    def _init_parameters(self) -> None:
        """Init scenario handler specific parameters from self._node"""
        self.MAP_PATH = self._get_param("general.map_path").string_value
        if not os.path.exists(self.MAP_PATH):
            raise ValueError("Can't find given map path: %s" % self.MAP_PATH)

        self.LEFT_DRIVING = self._get_param("scenario.left_driving").bool_value
        self.ADJACENCIES = self._get_param("scenario.adjacencies").bool_value
        self._dt = self._get_param("scenario.dt").double_value

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
        lanelet2_config.autoware = True

        scenario = lanelet_to_commonroad(
            input_file=map_filename,
            general_conf=general_config,
            lanelet2_conf=lanelet2_config,
        )

        scenario.dt = dt
        return scenario

    def _init_subscriptions(self) -> None:
        # subscribe predicted objects from perception
        _ = create_subscription(self._node,
                                spec_objects_sub,
                                lambda msg: self._last_msg.update({"dynamic_obstacle": msg}),
                                self._node.callback_group
                                )

    def _init_publishers(self) -> None:
        pass

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

    @property
    def z_coordinate(self) -> float:
        return self._z_coordinate

    def update_scenario(self):
        """
        Update the CommonRoad scenario using the perception/prediction input.
        """
        self._logger.info("Updating scenario")

        # process objects from perception
        self._process_objects()

        # print scenario update summary
        if self._VERBOSE:
            self._print_summary()

    def _print_summary(self):
        """
        Prints current scenario update to console via ROS logger for debugging
        """
        self._logger.debug(f"###### SCENARIO UPDATE")
        self._logger.debug(f"\t Number of dynamic obstacles: {len(self.scenario.dynamic_obstacles)}")
        self._logger.debug(f"\t Current obstacle types: "
                           f"{[obs.obstacle_type for obs in self.scenario.dynamic_obstacles]}")

    def _process_objects(self) -> None:
        """
        Converts Autoware objects to CommonRoad dynamic obstacles and add them to the CommonRoad scenario.
        The incoming objects are provided by the prediction module via the topic /perception/object_recognition/objects
        For each object we convert the estimated state and shape as well as the predicted trajectory

        Processing consists of three parts:
        - Adding newly appearing objects to the scenario
        - Updating existing objects (i.e., their state, shape and predicted trajectory)
        - Removing disappearing objects from the scenario
        """
        last_message = self._last_msg.get("dynamic_obstacle")  # message type: PredictedObjects
        if last_message is None:
            return

        for obstacle in last_message.objects:
            # convert current state
            position = map2utm(
                self._origin_transformation,
                obstacle.kinematics.initial_pose_with_covariance.pose.position,
            )
            orientation = quaternion2orientation(
                obstacle.kinematics.initial_pose_with_covariance.pose.orientation
            )
            velocity = obstacle.kinematics.initial_twist_with_covariance.twist.linear.x
            yaw_rate = obstacle.kinematics.initial_twist_with_covariance.twist.angular.z
            shape_type = obstacle.shape.type
            width = obstacle.shape.dimensions.y
            length = obstacle.shape.dimensions.x
            footprint = obstacle.shape.footprint
            classification = obstacle.classification
            time_step = 0

            # initialize list for predicted poses
            list_predicted_poses = []

            # get predicted path with highest confidence
            highest_conf_val = 0
            highest_conf_idx = 0
            for i in range(len(obstacle.kinematics.predicted_paths)):
                conf_val = obstacle.kinematics.predicted_paths[i].confidence
                if conf_val > highest_conf_val:
                    highest_conf_val = conf_val
                    highest_conf_idx = i

            # align time step of prediction to scenario time step
            obj_traj_dt = obstacle.kinematics.predicted_paths[highest_conf_idx].time_step.nanosec
            scenario_dt = self.scenario.dt * 1e9    # in nanoseconds

            if obj_traj_dt > scenario_dt:
                # upsample predicted path of obstacles to match dt
                self.upsample_predicted_path(
                    scenario_dt, obstacle, list_predicted_poses, obj_traj_dt, highest_conf_idx
                )
            elif obj_traj_dt < scenario_dt:
                # downsample predicted path of obstacles to match dt.
                self.downsample_predicted_path(
                    scenario_dt, obstacle, list_predicted_poses, obj_traj_dt, highest_conf_idx
                )
            else:
                # keep sampling of predicted path
                for pose in obstacle.kinematics.predicted_paths[highest_conf_idx].path:
                    list_predicted_poses.append(pose)
           
            # get Autoware object id
            object_id_aw: int = obstacle.object_id.uuid
            aw_id_list = [list(value) for value in self._dynamic_obstacles_map.values()]

            # process incoming objects
            if list(object_id_aw) not in aw_id_list:
                # newly appearing object: create new CommonRoad dynamic obstacle
                # current obstacle state
                dynamic_obstacle_initial_state = InitialState(
                    position=position,
                    orientation=orientation,
                    velocity=velocity,
                    acceleration=0.0,
                    yaw_rate=yaw_rate,
                    slip_angle=0.0,
                    time_step=time_step,
                )
                # generate unique CommonRoad ID add to ID mapping
                object_id_cr = self._scenario.generate_object_id()
                self._dynamic_obstacles_map[object_id_cr] = object_id_aw

                # add new dynamic obstacle to scenario
                self._add_dynamic_obstacle(
                    dynamic_obstacle_initial_state,
                    list_predicted_poses,
                    shape_type,
                    width,
                    length,
                    footprint,
                    classification,
                    object_id_cr,
                )
            else:
                # existing object: update state and shape
                for key, value in self._dynamic_obstacles_map.items():
                    if np.array_equal(object_id_aw, value):
                        # get obstacle with id
                        dynamic_obs = self._scenario.obstacle_by_id(key)

                        if isinstance(dynamic_obs, DynamicObstacle):
                            # shape update
                            aw_to_cr_shape_updater(
                                dynamic_obs, width, length, footprint
                            )

                            # state update
                            dynamic_obs.initial_state = InitialState(
                                position=position,
                                orientation=orientation,
                                velocity=velocity,
                                acceleration=0.0,
                                yaw_rate=yaw_rate,
                                slip_angle=0.0,
                                time_step=time_step,
                            )

                            # predicted trajectory update
                            if len(list_predicted_poses) > 2:
                                # update trajectory prediction for the obstacle
                                dynamic_obs.prediction = TrajectoryPrediction(
                                    self._pose_list_to_crtrajectory(
                                        dynamic_obs.initial_state.time_step + 1, list_predicted_poses
                                    ),
                                    dynamic_obs.obstacle_shape,
                                )

    def _pose_list_to_crtrajectory(self, time_step: int, list_poses: List[Pose]):
        """
        Converts a predicted obstacle path given as list of geometry_msgs/Pose into a CommonRoad trajectory type.
        :param time_step: initial time step of the input path
        :param list_poses: input path given as a list of geometry_msgs/Pose (i.e., positions and orientations)
        :return CRTrajectory: trajectory in the CommonRoad format
        """
        # CommonRoad state list
        cr_state_list = []

        # time step counter
        cnt_time_step = time_step

        for i in range(len(list_poses)):
            # transform position
            position = map2utm(self.origin_transformation, list_poses[i].position)
            # transform orientation
            orientation = quaternion2orientation(list_poses[i].orientation)
            # append state to CommonRoad state list
            cr_state = CustomState(position=position, orientation=orientation, time_step=cnt_time_step)
            cr_state_list.append(cr_state)
            # increment time step counter
            cnt_time_step += 1

        return CRTrajectory(time_step, cr_state_list)
                            
    @staticmethod
    def upsample_predicted_path(
        scenario_dt: float, 
        obstacle: PredictedObject,
        traj: List[Pose],
        obj_traj_dt: float,
        highest_conf_idx: int
    ) -> None:
        """
        Upsample predicted path of obstacles to match dt.
        
        :param scenario_dt: time step of the scenario
        :param obstacle: obstacle from autoware
        :param traj: trajectory of obstacle
        :param obj_traj_dt: time step of the obstacle
        :param highest_conf_idx: index of the highest confidence value in the predicted paths
        """
        if obj_traj_dt % scenario_dt == 0.0:
            dt_ratio = int(obj_traj_dt / scenario_dt) + 1
        else:
            dt_ratio = math.ceil(obj_traj_dt / scenario_dt)

        for point in obstacle.kinematics.predicted_paths[highest_conf_idx].path:
            traj.append(point)
            if len(traj) >= 2:
                upsample_trajectory(traj, dt_ratio)

    @staticmethod
    def downsample_predicted_path(
        scenario_dt: float,
        obstacle: PredictedObject,
        traj: List[Pose],
        obj_traj_dt: float,
        highest_conf_idx: int
    ) -> None:
        """
        Downsample predicted path of obstacles to match dt.

        :param scenario_dt: time step of the scenario
        :param obstacle: obstacle from autoware
        :param traj: trajectory of obstacle
        :param obj_traj_dt: time step of the obstacle
        :param highest_conf_idx: index of the highest confidence value in the predicted paths    
        """
        # if the time steps are divisible without reminder,
        # get the trajectories at the steps according to ratio
        if scenario_dt % obj_traj_dt == 0.0:
            dt_ratio = scenario_dt / obj_traj_dt
            for idx, point in enumerate(
                obstacle.kinematics.predicted_paths[highest_conf_idx].path
            ):
                if (idx + 1) % dt_ratio == 0:
                    traj.append(point)
        else:
            # make interpolation according to time steps
            dt_ratio = math.ceil(scenario_dt / obj_traj_dt)
            for idx, point in enumerate(
                obstacle.kinematics.predicted_paths[highest_conf_idx].path
            ):
                if (idx + 1) % dt_ratio == 0:
                    point_1 = obstacle.kinematics.predicted_paths[highest_conf_idx].path[
                        idx - 1
                    ]
                    point_2 = point
                    new_point = traj_linear_interpolate(
                        point_1, point_2, obj_traj_dt, scenario_dt
                    )
                    traj.append(new_point)

    def _add_dynamic_obstacle(
        self,
        initial_state: InitialState,
        traj: List[Pose],
        shape_type: int,
        width: float,
        length: float,
        footprint: PolygonMsg,
        classification: List[ObjectClassification],
        dynamic_obstacle_id: int,
    ) -> None:
        """
        Add dynamic obstacles with their predicted trajectory to the CommonRoad scenario.

        :param initial_state: initial state of obstacle
        :param traj: trajectory of obstacle
        :param shape_type: shape type of obstacle
        :param width: width of obstacle
        :param length: length of obstacle
        :param dynamic_obstacle_id: id of obstacle
        """
        # get initial time step
        time_step = initial_state.time_step

        # convert obstacle shape
        dynamic_obstacle_shape = aw_to_cr_shape(
            shape_type, width, length, footprint
        )

        # convert obstacle classification / type
        dynamic_obstacle_type = aw_to_cr_obstacle_type[
            get_classification_with_highest_probability(classification)
        ]

        if len(traj) > 2:
            # create the trajectory of the obstacle, starting at time_step
            dynamic_obstacle_trajectory = self._pose_list_to_crtrajectory(
                time_step + 1, traj
            )

            # create the prediction using the trajectory and the shape of the obstacle
            dynamic_obstacle_prediction = TrajectoryPrediction(
                dynamic_obstacle_trajectory, dynamic_obstacle_shape
            )

            # create the dynamic obstacle
            dynamic_obstacle = DynamicObstacle(
                dynamic_obstacle_id,
                dynamic_obstacle_type,
                dynamic_obstacle_shape,
                initial_state,
                dynamic_obstacle_prediction,
            )
        else:
            # create dynamic obstacle without a predicted trajectory
            dynamic_obstacle = DynamicObstacle(
                dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape, initial_state
            )

        # add dynamic obstacle to the scenario
        self.scenario.add_objects(dynamic_obstacle)

    def compute_z_coordinate(self, new_initial_pose: Optional[PoseWithCovarianceStamped],
                             new_goal_pose: Optional[PoseStamped]):
        """
        Approximation of the elevation (z-coordinate) of the scenario as the mean between initial_pose and goal_pose.

        :param new_initial_pose the initial pose of the ego vehicle
        :param new_goal_pose the desired goal pose
        """
        new_initial_pose_z = None
        new_goal_pose_z = None
        goal_pose_z = self._z_list[1]

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
            self._z_coordinate = 0.0
            self._logger.info("Z is either not found or is 0.0")
            return

        # set elevation as median between initial and goal pose z coordinates
        self._z_coordinate = float(np.median(valid_z_list))
