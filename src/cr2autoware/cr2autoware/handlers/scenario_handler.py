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
from uuid import UUID
import time

# third party
import numpy as np
from pyproj import Proj
import utm
import yaml
from shapely.geometry import Point, Polygon
from visualization_msgs.msg import Marker, MarkerArray

# ROS msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Point as PointMsg
from rclpy.publisher import Publisher
from rclpy.time import Time
from std_msgs.msg import Bool, Header

# Autoware msgs
from autoware_auto_perception_msgs.msg import PredictedObjects  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObject  # type: ignore
from autoware_auto_perception_msgs.msg import ObjectClassification  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedPath  # type: ignore
from autoware_perception_msgs.msg import TrafficSignalArray  # type: ignore
from autoware_perception_msgs.msg import TrafficSignal  # type: ignore
from autoware_perception_msgs.msg import TrafficSignalElement  # type: ignore

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
from ..common.utils.cr_conversion_utils import commonroad_shape_conversion
from ..common.utils.cr_conversion_utils import commonroad_shape_updater
from ..common.utils.cr_conversion_utils import commonroad_shape_to_marker
from ..common.utils.cr_conversion_utils import cr_obstacle_box_to_marker
from ..common.utils.cr_conversion_utils import get_classification_with_highest_probability
from ..common.utils.cr_conversion_utils import set_traffic_light_cycle
from ..common.utils.cr_conversion_utils import dict_autoware_to_cr_obstacle_type
from ..common.utils.cr_conversion_utils import dict_autoware_to_commonroad_traffic_light_color
from ..common.utils.cr_conversion_utils import dict_autoware_to_commonroad_traffic_light_shape
from ..common.utils.cr_conversion_utils import dict_autoware_to_commonroad_traffic_light_status
from ..common.utils.cr_conversion_utils import uuid_from_ros_msg
from ..common.utils.transform import quaternion2orientation
from ..common.utils.transform import map2utm
from ..common.utils.geometry import upsample_trajectory
from ..common.utils.geometry import traj_linear_interpolate
from ..common.ros_interface.create import create_subscription
from ..common.ros_interface.create import create_publisher
from ..interfaces.implementation.behavior_tree.behavior_utils import BehaviorScenarioParams

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto

# subscriber specifications
from ..common.ros_interface.specs_subscriptions import spec_objects_sub
from ..common.ros_interface.specs_subscriptions import spec_traffic_signals_sub
from ..common.ros_interface.specs_subscriptions import spec_simulated_traffic_light_sub

#publisher specifications
from ..common.ros_interface.specs_publisher import spec_cr_obstacles_pub
from ..common.ros_interface.specs_publisher import spec_cr_obstacle_box_pub


class ScenarioHandler(BaseHandler):
    """
    Handles communication with Autoware for CommonRoad Scenario relevant data.

    Keeps an up-to-date state of the current scenario in CommonRoad format.

    ----------------
    **Publishers:**

    * spec_cr_obstacles_pub: 
        * Description: CommonRoad scenario obstacles.
        * Topic: `/planning/commonroad/cr_obstacles`
        * Message Type: `visualization_msgs.msg.MarkerArray`
    * spec_cr_obstacle_box_pub:
        * Description: CommonRoad scenario box for obstacles.
        * Topic: `/planning/commonroad/cr_obstacle_box`
        * Message Type: `visualization_msgs.msg.MarkerArray`

    -------------------
    **Subscribers:**

    * spec_objects_sub:
        * Description: Subscribes to predicted objects from perception
        * Topic: `/perception/object_recognition/objects`
        * Message Type: `autoware_auto_perception_msgs.msg.PredictedObject`
    * spec_traffic_signals_sub:
        * Description: Subscribes to traffic lights from perception
        * Topic: `/perception/traffic_light_recognition/traffic_signals`
        * Message Type: `autoware_perception_msgs.msg.TrafficSignalArray`
    * spec_simulated_traffic_light_sub:
        * Description: Subscribes to simulated traffic light
        * Topic: `/planning/commonroad/test_mode/traffic_light`
        * Message Type: `std_msgs.msg.Bool`

    -------------------
    :var MAP_PATH: path to the map directory containing the map_config.yaml file
    :var LEFT_DRIVING: flag indicating if the map is for left driving
    :var ADJACENCIES: flag indicating if the map contains adjacencies
    :var _scenario: CommonRoad scenario
    :var _dt: time step of the scenario
    :var _origin_transformation: origin transformation between CommonRoad and Autoware map
    :var _last_msg: last received message from perception
    :var _object_id_mapping: mapping between Autoware object ID and CommonRoad object ID
    :var _road_boundary: static road boundary collision object
    :var _z_list: list of z-coordinates of initial and goal pose
    :var _initialpose3d_z: z-coordinate of initial pose
    :var _z_coordinate: current elevation (z-coordinate)
    """

    # Constants and parameters
    MAP_PATH: str
    LEFT_DRIVING: bool
    ADJACENCIES: bool

    _scenario: CRScenario
    _dt: float
    _origin_transformation: List[float]
    _last_msg: Dict[str, Any] = {}

    # mapping between Autoware object ID (key, given as UUID)
    # and corresponding CommonRoad object ID (value, given as int)
    _object_id_mapping: Dict[UUID, int] = {}

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

        # Initialze params for traffic light simulation
        self._simulated_green_light = False
        self._yellow_time = None

    def _init_parameters(self) -> None:
        """
        Init scenario handler specific parameters from self._node

        :raises ValueError: if the map path is not found
        :raises VauleError: if safety margin is negative
        """
        self.MAP_PATH = self._get_param("general.map_path").string_value
        if not os.path.exists(self.MAP_PATH):
            raise ValueError("Can't find given map path: %s" % self.MAP_PATH)

        self.LEFT_DRIVING = self._get_param("scenario.left_driving").bool_value
        self.ADJACENCIES = self._get_param("scenario.adjacencies").bool_value
        self._dt = self._get_param("scenario.dt").double_value
        # safety margin for obstacle shapes
        self.safety_margin = self._get_param("trajectory_planner.safety_margin").double_value
        if self.safety_margin < 0.0:
            raise ValueError("Safety margin must be greater or equal to 0.0!")
        # flag indicating if ROS topics visualizing the CR scenario should be published
        self.publish_cr_scenario_topics = self._get_param("scenario.publish_cr_scenario_topics").bool_value
        # CR obstacle box dimensions for scenario objects from the ego vehicle
        self.cr_obstacle_box_front = self._get_param("scenario.cr_obstacle_box_front").double_value
        self.cr_obstacle_box_rear = self._get_param("scenario.cr_obstacle_box_rear").double_value
        self.cr_obstacle_box_side = self._get_param("scenario.cr_obstacle_box_side").double_value
        self.cr_obstacle_box_prediction = self._get_param("scenario.cr_obstacle_box_prediction").bool_value

        # flag for traffic light simulation in test drive
        self.test_mode_traffic_light = self._get_param("scenario.test_mode_traffic_light").bool_value
        self.yellow_light_time = self._get_param("behavior_planner.yellow_light_time").double_value

        # only consider currently perceived traffic lights from perception (others are set to inactive)
        self.only_current_traffic_lights = self._get_param("scenario.only_current_traffic_lights").bool_value

    def _read_map_config(self, map_path: str) -> Dict[str, Any]:
        """
        Read the map config file to obtain the origin (lat/lon) of the local coordinates of the lanelet2 map.

        :param map_path: path to directory containing the map_config.yaml file
        :return: map configuration dictionary
        :raises ValueError: if map origin can't be loaded and no YAML file exists in the map directory
        :raises _logger.warn: if map origin is invalid (i.e., lat/lon values are out of range)
        :raises KeyError: if `use_local_coordinates` parameter is not found in the map_config.yaml file
        """
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
        """
        Obtain the projection string based on the lat/lon origin of the Lanelet2 map.

        :param map_config: map configuration dictionary
        :return: projection string
        """
        aw_origin_latitude = map_config["aw_origin_latitude"]
        aw_origin_longitude = map_config["aw_origin_longitude"]
        utm_str = utm.from_latlon(float(aw_origin_latitude), float(aw_origin_longitude))
        projection_string = "+proj=utm +zone=%d +datum=WGS84 +ellps=WGS84" % (utm_str[2])
        self._logger.debug(f"Proj string: {projection_string}")
        return projection_string

    def _get_origin_transformation(
        self, map_config: Dict[str, Any], projection: Proj, scenario: CRScenario
    ) -> List[Union[float, Any]]:
        """
        Compute the origin transformation.

        (i.e., translation between the local coordinates of the CommonRoad map and the Autoware Lanelet2 map)

        :param map_config: map configuration dictionary
        :param projection: projection string
        :param scenario: CommonRoad scenario
        :return: origin transformation
        """
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
        """
        Initialize the scenario either from a CommonRoad scenario file or from Autoware map.

        Transforms Autoware map to CommonRoad scenario if no CommonRoad scenario file is found  in the `map_path`
        directory.

        :param map_path: path to the map directory containing the map_config.yaml file
        :param projection_string: projection string
        :param left_driving: flag indicating if the map is for left driving
        :param adjacencies: flag indicating if the map contains adjacencies
        :param dt: time step of the scenario
        :return: CommonRoad scenario
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

        :return: road boundary collision object
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
        """
        Build the CommonRoad scenario from a CommonRoad scenario file.

        :param map_filename_cr: CommonRoad scenario file
        :return: CommonRoad scenario
        """
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
        """
        Build the CommonRoad scenario from an Autoware map.

        :param map_filename: Autoware map file
        :param projection_string: projection string
        :param left_driving: flag indicating if the map is for left driving
        :param adjacencies: flag indicating if the map contains adjacencies
        :param dt: time step of the scenario
        :return: CommonRoad scenario
        """
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
        """Initialize subscriptions."""
        # subscribe predicted objects from perception
        _ = create_subscription(self._node,
                                spec_objects_sub,
                                lambda msg: self._last_msg.update({"dynamic_obstacle": msg}),
                                self._node.callback_group
                                )
        # subscribe traffic lights from perception
        _ = create_subscription(self._node,
                                spec_traffic_signals_sub,
                                lambda msg: self._last_msg.update({"traffic_lights": msg}),
                                self._node.callback_group
                                )
        # subscribe to simulated traffic light topic
        if self.test_mode_traffic_light:
            _ = create_subscription(self._node,
                                    spec_simulated_traffic_light_sub,
                                    self.simulated_traffic_light_callback,
                                    self._node.callback_group
                                    )

    def _init_publishers(self) -> None:
        """Initialize publishers."""
        # publish CommonRoad scenario obstacles
        self._pub_cr_obstacles = create_publisher(self._node, spec_cr_obstacles_pub)

        # publish CommonRoad scenario box for obstacles
        self._pub_cr_obstacle_box = create_publisher(self._node, spec_cr_obstacle_box_pub)

    @property
    def scenario(self) -> CRScenario:
        """
        Getter for current CR scenario.

        :return: CommonRoad scenario
        """
        return self._scenario

    @property
    def lanelet_network(self) -> LaneletNetwork:
        """
        Getter for CR lanelet network.

        :return: lanelet network
        """
        return self._scenario.lanelet_network

    @property
    def road_boundary(self) -> pycrcc.CollisionObject:
        """
        Getter for road boundary collision object.

        :return: road boundary collision object
        """
        return self._road_boundary

    @property
    def origin_transformation(self) -> List[Union[float, Any]]:
        """
        Getter for the origin transformation.

        :return: origin transformation
        """
        return self._origin_transformation

    @property
    def z_coordinate(self) -> float:
        """
        Getter for the elevation / z-coordinate.

        :return: elevation (z-coordinate)
        """
        return self._z_coordinate
    
    @property
    def time_step(self) -> float:
        """
        Getter for the time step of the scenario.

        :return: time step
        """
        return self._dt
    
    @property
    def ros_time(self) -> Time:
        """
        Getter for the current ROS time.

        :return: current ROS time
        """
        return self._node.get_clock().now().to_msg()

    def update_scenario(self, behavior_scenario_params: BehaviorScenarioParams) -> None:
        """Update the CommonRoad scenario using the perception/prediction input."""
        if self._VERBOSE:
            self._logger.info("Updating scenario")

        # log time
        t_start = time.perf_counter()

        # process objects from perception
        self._process_objects(behavior_scenario_params)
        self._logger.debug(f"[SVEN] [TIME] Processing objects took: {time.perf_counter() - t_start} s")

        t_tl = time.perf_counter()
        # process traffic lights from perception
        self._process_traffic_lights()
        self._logger.debug(f"[SVEN] [TIME] Processing traffic lights took: {time.perf_counter() - t_tl} s")


        t_assigned = time.perf_counter()
        # assign objects to lanelets
        # obstacle_ids = {obs.obstacle_id for obs in self.scenario.dynamic_obstacles}
        # self._scenario.assign_obstacles_to_lanelets(obstacle_ids=obstacle_ids)
        self._logger.debug(f"[SVEN] [TIME] Assigning obstacles to lanelets took: {time.perf_counter() - t_assigned} s")

        # log time
        t_elapsed = time.perf_counter() - t_start

        # print scenario update summary
        time_print = time.perf_counter()
        if self._VERBOSE:
            self._print_summary(t_elapsed)
        self._logger.debug(f"[SVEN] [TIME] Printing summary took: {time.perf_counter() - time_print} s")

    def _print_summary(self, t_elapsed: float) -> None:
        """
        Prints current scenario update to console via ROS logger for debugging

        :param t_elapsed: elapsed scenario update time
        """
        self._logger.debug(f"###### SCENARIO UPDATE")
        self._logger.debug(f"\t Scenario update took: {t_elapsed} s")
        self._logger.debug(f"\t Number of dynamic obstacles: {len(self.scenario.dynamic_obstacles)}")
        self._logger.debug(f"\t Current obstacle types: "
                           f"{[obs.obstacle_type.value for obs in self.scenario.dynamic_obstacles]}")
        self._logger.debug(f"\t Current CR obstacle IDs: "
                           f"{[obs.obstacle_id for obs in self.scenario.dynamic_obstacles]}")
        self._logger.debug(f"\t Current CR traffic light IDs: "
                           f"{[tl.traffic_light_id for tl in self.lanelet_network.traffic_lights if tl.active is True]}")

    def _process_objects(self, behavior_scenario_params: BehaviorScenarioParams) -> None:
        """
        Converts Autoware objects to CommonRoad dynamic obstacles and add them to the CommonRoad scenario.

        The incoming objects are provided by the prediction module via the topic:
        `/perception/object_recognition/objects`
        For each object we convert the estimated state and shape as well as the predicted trajectory.

        Processing consists of three parts:
        * Adding newly appearing objects to the scenario
        * Updating existing objects (i.e., their state, shape and predicted trajectory)
        * Removing disappearing objects from the scenario

        :param behavior_scenario_params: scenario parameters set by the behavior planner
        """
        last_message = self._last_msg.get("dynamic_obstacle")  # message type: PredictedObjects
        if last_message is None:
            return

        # get list of previously registered AW object IDs in CR scenario
        list_prev_aw_object_ids: List[UUID] = list(self._object_id_mapping.keys())

        # list of AW object IDs in current perception message
        list_curr_aw_object_ids: List[UUID] = list()

        # create CR obstacle box polygon around ego vehicle for object filtering
        cr_obstacle_box = self._cr_obstacle_box(behavior_scenario_params)

        for obstacle in last_message.objects:
            # check if obstacle is within the ego vehicle's CR obstacle box
            if not self._is_in_cr_obstacle_box(cr_obstacle_box, obstacle, behavior_scenario_params):
                continue

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
            width = obstacle.shape.dimensions.y
            length = obstacle.shape.dimensions.x
            footprint = obstacle.shape.footprint
            classification = obstacle.classification
            time_step = 0

            # initialize list for predicted poses
            list_predicted_poses = []

            # get predicted path with highest confidence
            object_predicted_path: PredictedPath = self._get_predicted_path(obstacle)

            # align time step of prediction to scenario time step
            obj_traj_dt = object_predicted_path.time_step.nanosec
            scenario_dt = self.scenario.dt * 1e9    # in nanoseconds

            if obj_traj_dt > scenario_dt:
                # upsample predicted path of obstacles to match dt
                self._upsample_predicted_path(
                    scenario_dt, object_predicted_path, list_predicted_poses, obj_traj_dt
                )
            elif obj_traj_dt < scenario_dt:
                # downsample predicted path of obstacles to match dt.
                self._downsample_predicted_path(
                    scenario_dt, object_predicted_path, list_predicted_poses, obj_traj_dt,
                )
            else:
                # keep sampling of predicted path
                for pose in object_predicted_path.path:
                    list_predicted_poses.append(pose)

            # get AW object ID: convert ROS2 UUID msg to Python UUID
            object_id_aw: UUID = uuid_from_ros_msg(obstacle.object_id.uuid)

            # append AW object ID to current list
            list_curr_aw_object_ids.append(object_id_aw)

            # process incoming objects
            if object_id_aw not in list_prev_aw_object_ids:
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
                self._object_id_mapping[object_id_aw] = object_id_cr

                # add new dynamic obstacle to scenario
                self._add_dynamic_obstacle(
                    dynamic_obstacle_initial_state,
                    list_predicted_poses,
                    width,
                    length,
                    footprint,
                    classification,
                    object_id_cr,
                )
            else:
                # existing object: update its state, shape and prediction
                # get CR object ID from mapping
                object_id_cr = self._object_id_mapping[object_id_aw]

                # get obstacle with id
                dynamic_obs = self._scenario.obstacle_by_id(object_id_cr)

                # shape update
                commonroad_shape_updater(
                    dynamic_obs, width, length, footprint, self.safety_margin
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

        # remove obstacles from scenario which are not in the current objects message
        self._remove_objects_from_scenario(list_curr_aw_object_ids)

        if self.publish_cr_scenario_topics:
            # publish CR obstacle box
            self._publish_cr_obstacle_box(cr_obstacle_box)
            # publish CommonRoad obstacles
            self._publish_cr_obstacles(self._scenario.obstacles)

    def _remove_objects_from_scenario(self, list_curr_aw_object_ids: List[UUID]) -> None:
        """
        Object removal from the CommonRoad scenario.

        Removes all objects from the CR scenario which are not present in the current objects message from the AW
        perception module. Objects are removed from the CR scenario and from the object ID mapping

        :param list_curr_aw_object_ids: List of AW object IDs in the current perception message
        """
        # init list to store removed AW object IDs
        list_removed_aw_object_ids = list()

        # iterate over AW object IDs in mapping
        for key in self._object_id_mapping.keys():
            if key not in list_curr_aw_object_ids:
                # get CR object ID
                cr_object_id = self._object_id_mapping[key]
                # remove obstacle from scenario
                self._scenario.remove_obstacle(self._scenario.obstacle_by_id(cr_object_id))

                # add to list of removed IDs
                list_removed_aw_object_ids.append(key)

        # remove from object ID mapping
        for idx in list_removed_aw_object_ids:
            self._object_id_mapping.pop(idx)

    def _cr_obstacle_box(self, behavior_scenario_params: BehaviorScenarioParams) -> Polygon:
        """
        Get the CR obstacle box of the ego vehicle as a polygon.

        :param behavior_scenario_params: behavior scenario parameters
        :return: CR obstacle box
        """
        # get ego vehicle position and orientation
        ego_vehicle_position = self._node.ego_vehicle_handler.current_vehicle_state.pose.pose.position
        ego_vehicle_quaternion = self._node.ego_vehicle_handler.current_vehicle_state.pose.pose.orientation

        # convert quaternion to orientation
        ego_vehicle_orientation = quaternion2orientation(ego_vehicle_quaternion)

        # calculate the corners of the perception rectangle
        cos_theta = math.cos(ego_vehicle_orientation)
        sin_theta = math.sin(ego_vehicle_orientation)

        # set bounding box dimensions
        # if behavior planner requires a different bounding box size, set it here
        if behavior_scenario_params.cr_obstacle_box_front is not None:
            box_front_dim = behavior_scenario_params.cr_obstacle_box_front
        else: 
            box_front_dim = self.cr_obstacle_box_front

        if behavior_scenario_params.cr_obstacle_box_rear is not None:
            box_rear_dim = behavior_scenario_params.cr_obstacle_box_rear
        else:
            box_rear_dim = self.cr_obstacle_box_rear

        if behavior_scenario_params.cr_obstacle_box_side is not None:
            box_side_dim = behavior_scenario_params.cr_obstacle_box_side
        else:
            box_side_dim = self.cr_obstacle_box_side


        front_left = Point(
            ego_vehicle_position.x + box_front_dim * cos_theta - box_side_dim * sin_theta,
            ego_vehicle_position.y + box_front_dim * sin_theta + box_side_dim * cos_theta
        )
        front_right =  Point(
            ego_vehicle_position.x + box_front_dim * cos_theta + box_side_dim * sin_theta,
            ego_vehicle_position.y + box_front_dim * sin_theta - box_side_dim * cos_theta
        )
        rear_left = Point(
            ego_vehicle_position.x - box_rear_dim * cos_theta - box_side_dim * sin_theta,
            ego_vehicle_position.y - box_rear_dim * sin_theta + box_side_dim * cos_theta
        )
        rear_right = Point(
            ego_vehicle_position.x - box_rear_dim * cos_theta + box_side_dim * sin_theta,
            ego_vehicle_position.y - box_rear_dim * sin_theta - box_side_dim * cos_theta
        )

        # create a polygon for the perception 
        perception_polygon = Polygon([rear_left, front_left, front_right, rear_right])

        return perception_polygon
    
    def _publish_cr_obstacle_box(self, cr_obstacle_box: Polygon) -> None:
        """
        Publish the CR obstacle box as a MarkerArray message.

        :param cr_obstacle_box: CR obstacle box of ego vehicle
        """
        marker_array = MarkerArray()

        time_stamp = self._node.get_clock().now().to_msg()
        marker = cr_obstacle_box_to_marker(cr_obstacle_box, self._z_coordinate, time_stamp)
        marker_array.markers.append(marker)

        # publish CR obstacle box
        self._pub_cr_obstacle_box.publish(marker_array)

    def _publish_cr_obstacles(self, obstacles: List) -> None:
        """
        Publish CommonRoad obstacles as a MarkerArray message.

        :param obstacles: list of CommonRoad obstacles
        """
        marker_array = MarkerArray()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        time_stamp = self._node.get_clock().now().to_msg()

        for obstacle in obstacles:
            marker = commonroad_shape_to_marker(obstacle, self._origin_transformation, self._z_coordinate, time_stamp)
            marker_array.markers.append(marker)

        # publish obstacles
        self._pub_cr_obstacles.publish(marker_array)

    def _is_in_cr_obstacle_box(self, cr_obstacle_box: Polygon, obstacle: PredictedObject, behavior_scenario_params: BehaviorScenarioParams) -> bool:
        """
        Check if the given obstacle position is within the ego vehicle's or prediction's CR obstacle box.

        :param cr_obstacle_box: CR obstacle box of ego vehicle
        :param obstacle: predicted object from perception
        :return: True if the position is within the CR obstacle box, False otherwise
        """
        obs_position = obstacle.kinematics.initial_pose_with_covariance.pose.position   
        obstacle_point = Point(obs_position.x, obs_position.y)

        # check if obstacle is within the CR obstacle box
        if cr_obstacle_box.contains(obstacle_point):
            return True

        # get flag for checking the predicted path
        if behavior_scenario_params.cr_obstacle_box_prediction is not None:
            check_prediction = behavior_scenario_params.cr_obstacle_box_prediction
        else:
            check_prediction = self.cr_obstacle_box_prediction
        # check if predicted poses are within the CR obstacle box
        if check_prediction:
            predicted_path: PredictedPath = self._get_predicted_path(obstacle)
            for pose in predicted_path.path:
                obstacle_point = Point(pose.position.x, pose.position.y)
                if cr_obstacle_box.contains(obstacle_point):
                    return True

        return False

    @staticmethod
    def _get_predicted_path(predicted_object: PredictedObject) -> PredictedPath:
        """
        Retrieves the predicted path of an object.

        If multiple predicted paths are available, the prediction with the highest confidence is returned

        :param predicted_object: predicted object from perception
        :return: predicted path with the highest confidence
        """
        highest_conf_val = 0
        highest_conf_idx = 0
        for i in range(len(predicted_object.kinematics.predicted_paths)):
            conf_val = predicted_object.kinematics.predicted_paths[i].confidence
            if conf_val > highest_conf_val:
                highest_conf_val = conf_val
                highest_conf_idx = i

        return predicted_object.kinematics.predicted_paths[highest_conf_idx]

    def _pose_list_to_crtrajectory(self, time_step: int, list_poses: List[Pose]) -> CRTrajectory:
        """
        Converts a predicted obstacle path given as list of `geometry_msgs/Pose` into a CommonRoad trajectory type.

        :param time_step: initial time step of the input path
        :param list_poses: input path given as a list of `geometry_msgs/Pose` (i.e., positions and orientations)
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
    def _upsample_predicted_path(
        scenario_dt: float, 
        object_predicted_path: PredictedPath,
        return_path: List[Pose],
        obj_traj_dt: float,
    ) -> None:
        """
        Upsample predicted path of obstacles to match time step dt of the scenario.
        
        :param scenario_dt: time step of the scenario
        :param object_predicted_path: predicted path of the object from AW prediction
        :param return_path: upsampled predicted path of the object
        :param obj_traj_dt: time step of the obstacle
        """
        if obj_traj_dt % scenario_dt == 0.0:
            dt_ratio = int(obj_traj_dt / scenario_dt) + 1
        else:
            dt_ratio = math.ceil(obj_traj_dt / scenario_dt)

        for point in object_predicted_path.path:
            return_path.append(point)
            if len(return_path) >= 2:
                upsample_trajectory(return_path, dt_ratio)

    @staticmethod
    def _downsample_predicted_path(
        scenario_dt: float,
        object_predicted_path: PredictedPath,
        return_path: List[Pose],
        obj_traj_dt: float,
    ) -> None:
        """
        Downsample predicted path of obstacles to match time step dt of the scenario.

        :param scenario_dt: time step of the scenario
        :param object_predicted_path: predicted path of the object from AW prediction
        :param return_path: downsampled predicted path of the object
        :param obj_traj_dt: time step of the obstacle
        """
        # if the time steps are divisible without remainder,
        # get the trajectories at the steps according to ratio
        if scenario_dt % obj_traj_dt == 0.0:
            dt_ratio = scenario_dt / obj_traj_dt
            for idx, point in enumerate(
                object_predicted_path.path
            ):
                if (idx + 1) % dt_ratio == 0:
                    return_path.append(point)
        else:
            # make interpolation according to time steps
            dt_ratio = math.ceil(scenario_dt / obj_traj_dt)
            for idx, point in enumerate(
                object_predicted_path.path
            ):
                if (idx + 1) % dt_ratio == 0:
                    point_1 = object_predicted_path.path[
                        idx - 1
                    ]
                    point_2 = point
                    new_point = traj_linear_interpolate(
                        point_1, point_2, obj_traj_dt, scenario_dt
                    )
                    return_path.append(new_point)

    def _add_dynamic_obstacle(
        self,
        initial_state: InitialState,
        traj: List[Pose],
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
        :param width: width of obstacle
        :param length: length of obstacle
        :param footprint: footprint of obstacle
        :param classification: classification of obstacle
        :param dynamic_obstacle_id: id of obstacle
        """
        # get initial time step
        time_step = initial_state.time_step

        # convert obstacle classification / type
        dynamic_obstacle_type = dict_autoware_to_cr_obstacle_type[
            get_classification_with_highest_probability(classification)
        ]

        # create shape of the obstacle
        dynamic_obstacle_shape = commonroad_shape_conversion(
            dynamic_obstacle_type, width, length, footprint, self.safety_margin
        )

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

    def simulated_traffic_light_callback(self, msg: Bool) -> None:
        """
        Callback function for the simulated traffic light topic.

        :param msg: simulated traffic light message
        """
        is_green = msg.data

        if is_green:
            # set traffic light to green
            self._simulated_green_light = True
            self._yellow_time = None

        else:
            # save the time when the traffic light turned yellow (if it was green before)
            if self._simulated_green_light:
                self._yellow_time = self._node.get_clock().now()
            self._simulated_green_light = False

    def _process_traffic_lights(self) -> None:
        """
        Converts Autoware traffic lights to CommonRoad traffic lights and updates the CommonRoad scenario.

        There are two different traffic light conversion modes:
        
        * Simulation: The traffic lights are converted by a boolean value via the topic: `/planning/commonroad/test_mode/traffic_light`
        * Perception: The incoming traffic lights are provided by the perception module via the topic: `/perception/traffic_light_recognition/traffic_signals`
        """

        last_message = self._last_msg.get("traffic_lights") # type: TrafficSignalArray

        self._logger.debug("[SVEN] [TrafficLights] Processing traffic lights: " + str(last_message))

        # process traffic lights from simulation
        if self.test_mode_traffic_light:
            # set color and cycle depending on the simulated traffic light state
            if self._simulated_green_light:
                color_cr = dict_autoware_to_commonroad_traffic_light_color[3]
            else:
                # if the traffic light is not green, check if yellow time is set
                # set traffic light to red if no yellow time is set (no message received)
                if self._yellow_time is None:

                    color_cr = dict_autoware_to_commonroad_traffic_light_color[1]
                
                # if yellow time is under 3 seconds, set traffic light to yellow
                elif (self._node.get_clock().now() - self._yellow_time).nanoseconds < (self.yellow_light_time * 1e9):
                    color_cr = dict_autoware_to_commonroad_traffic_light_color[2]
                
                # if yellow time is over 3 seconds, set traffic light to red
                else:
                    color_cr = dict_autoware_to_commonroad_traffic_light_color[1]

            # use /planning/commonroad/test_mode/traffic_light topic for traffic light simulation
            for traffic_light_ln in self.lanelet_network.traffic_lights:
                # set traffic light active state to True
                traffic_light_ln.active = True
                traffic_light_ln.color = color_cr
                traffic_light_ln.traffic_light_cycle = set_traffic_light_cycle(color_cr)
                traffic_light_ln.direction = dict_autoware_to_commonroad_traffic_light_shape[1]

        # process traffic lights from perception
        else: 

            if last_message is None:
                return

            # initialize list of processed traffic light IDs
            processed_traffic_light_ids: List[int] = []

            # process all traffic lights from perception message
            for traffic_signal in last_message.signals:
                # get traffic light ID
                traffic_signal_id = traffic_signal.traffic_signal_id

                # add traffic light ID to processed list
                processed_traffic_light_ids.append(traffic_signal_id)

                # get traffic light from lanelet network
                traffic_light_cr = self.lanelet_network.find_traffic_light_by_id(traffic_signal_id)

                # get traffic light element with the highest confidence
                traffic_light: TrafficSignalElement = self._get_traffic_light(traffic_signal)

                # get traffic light status, color and shape
                status = traffic_light.status
                color = traffic_light.color
                shape = traffic_light.shape

                # convert traffic light status
                try:
                    status_cr = dict_autoware_to_commonroad_traffic_light_status[status]
                except KeyError:
                    self._logger.error("Traffic light status not found in AW to CR status map!")
                    continue

                # check if detected traffic light is active:
                if status_cr is True:
                    # set traffic light color and cycle
                    try:
                        color_cr = dict_autoware_to_commonroad_traffic_light_color[color]
                    except KeyError:
                        self._logger.error("Traffic light color not found in AW to CR color map!")
                        continue
                    traffic_light_cr.color = color_cr
                    traffic_light_cr.traffic_light_cycle = set_traffic_light_cycle(color_cr)

                    # set traffic light direction
                    try:
                        shape_cr = dict_autoware_to_commonroad_traffic_light_shape[shape]
                    except KeyError:
                        self._logger.error("Traffic light shape not found in AW to CR shape map!")
                        continue
                    traffic_light_cr.direction = shape_cr
                else:
                    # set traffic light color and cycle to inactive
                    color_cr = dict_autoware_to_commonroad_traffic_light_color[99]
                    traffic_light_cr.color = color_cr
                    traffic_light_cr.traffic_light_cycle = set_traffic_light_cycle(color_cr)
                    traffic_light_cr.direction = dict_autoware_to_commonroad_traffic_light_shape[shape]

                # set detected traffic light in perception message to active
                traffic_light_cr.active = True

            # set traffic light active state to False and traffic light cycle to inactive for all traffic lights that are not in the perception message
            if self.only_current_traffic_lights:
                for traffic_light_ln in self.lanelet_network.traffic_lights:
                    if traffic_light_ln.active is True:
                        if traffic_light_ln.traffic_light_id not in processed_traffic_light_ids:
                            traffic_light_ln.active = False
                            color_inactive = dict_autoware_to_commonroad_traffic_light_color[99]
                            traffic_light_ln.traffic_light_cycle = set_traffic_light_cycle(color_inactive)

    @staticmethod
    def _get_traffic_light(traffic_signal: TrafficSignal) -> TrafficSignalElement:
        """
        Retrieves the traffic light with the highest confidence value from the perception message.

        If multiple traffic lights are available, the traffic light with the highest confidence is returned.

        :param traffic_signal: traffic signal message from perception
        :return TrafficLight: traffic light with the highest confidence
        """
        highest_conf_val = 0
        highest_conf_idx = 0
        for i in range(len(traffic_signal.elements)):
            conf_val = traffic_signal.elements[i].confidence
            if conf_val > highest_conf_val:
                highest_conf_val = conf_val
                highest_conf_idx = i

        return traffic_signal.elements[highest_conf_idx]

    def compute_z_coordinate(self, new_initial_pose: Optional[PoseWithCovarianceStamped],
                             new_goal_pose: Optional[PoseStamped]) -> None:
        """
        Approximation of the elevation (z-coordinate) of the scenario as the mean between initial_pose and goal_pose.

        :param new_initial_pose: the initial pose of the ego vehicle
        :param new_goal_pose: the desired goal pose
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
