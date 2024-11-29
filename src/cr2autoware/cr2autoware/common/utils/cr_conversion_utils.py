"""
CR to Autoware Conversion Utils
=============================

This module provides functions to convert data from CommonRoad to Autoware and vice versa.

-----------------------
**Dictionaries:**

* dict_autoware_to_cr_obstacle_type: Dict[int, ObstacleType]
    * Description: Dictionary to map Autoware obstacle ids to CommonRoad obstacle types.
    * Key: Autoware obstacle id
    * Value: CommonRoad obstacle type
    * Mapping:
        * 0: UNKNOWN
        * 1: CAR
        * 2: TRUCK
        * 3: BUS
        * 4: TRUCK
        * 5: MOTORCYCLE
        * 6: BICYCLE
        * 7: PEDESTRIAN

* dict_autoware_to_commonroad_traffic_light_color: Dict[int, TrafficLightState]
    * Description: Dictionary to map Autoware traffic light colors to CommonRoad traffic light states   .
    * Key: Autoware traffic light color
    * Value: CommonRoad traffic light state
    * Mapping:
        * 1: RED
        * 2: YELLOW
        * 3: GREEN
        * 18: UNKNOWN
        * 4: WHITE
        * Additional states CommonRoad:
            * TrafficLightState.RED_YELLOW
            * TrafficLightState.INACTIVE

* dict_autoware_to_commonroad_traffic_light_shape: Dict[int, TrafficLightDirection]
    * Description: Dictionary to map Autoware traffic light shapes to CommonRoad traffic light directions.
    * Key: Autoware traffic light shape
    * Value: CommonRoad traffic light direction
    * Mapping:
        * 5: CIRCLE
        * 6: LEFT_ARROW
        * 7: RIGHT_ARROW
        * 8: UP_ARROW
        * 9: UP_LEFT_ARROW
        * 10: UP_RIGHT_ARROW
        * Additional states Autoware:
            * 11: DOWN_ARROW
            * 12: DOWN_LEFT_ARROW
            * 13: DOWN_RIGHT_ARROW
            * 18: UNKNOWN
            * 0: CROSS
        * Additional states CommonRoad:
            * TrafficLightDirection.LEFT_RIGHT


* dict_autoware_to_commonroad_traffic_light_status: Dict[int, bool]
    * Description: Dictionary to map Autoware traffic light status to CommonRoad traffic light status.
    * Key: Autoware traffic light status
    * Value: CommonRoad traffic light status
    * Mapping:
        * 15: SOLID_OFF
        * 16: SOLID_ON
-----------------------
"""
# standard imports
import math
import os
from uuid import UUID as PyUUID

# third party imports
import numpy as np
from shapely.geometry import Polygon as PolygonShapely

# ROS imports
import rclpy.logging as ros_logging # type: ignore

# comonroad-io imports
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState,CustomState
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightState, TrafficLightDirection, \
    TrafficLightCycleElement, TrafficLightCycle
from commonroad.geometry.shape import Shape, Rectangle, Circle, Polygon


# Third party
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_planning_msgs.msg import TrajectoryPoint  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects, ObjectClassification, PredictedPath  # type: ignore
from autoware_auto_perception_msgs.msg import TrafficSignalArray # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from geometry_msgs.msg import Pose # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
from geometry_msgs.msg import Polygon as PolygonMsg # type: ignore


# own code base
import cr2autoware.common.utils.transform as transform_utils

# typing
from typing import List, Dict, Tuple, Union

# ROS msgs
from unique_identifier_msgs.msg import UUID as UUIDMsg


logger = ros_logging.get_logger(__name__)


dict_autoware_to_cr_obstacle_type: Dict[int, ObstacleType] = {
    0: ObstacleType.UNKNOWN,
    1: ObstacleType.CAR,
    2: ObstacleType.TRUCK,
    3: ObstacleType.BUS,
    4: ObstacleType.TRUCK,
    5: ObstacleType.MOTORCYCLE,
    6: ObstacleType.BICYCLE,
    7: ObstacleType.PEDESTRIAN,
}


dict_autoware_to_commonroad_traffic_light_color: Dict[int, TrafficLightState] = {
    1: TrafficLightState.RED,  # AW: RED
    2: TrafficLightState.YELLOW,  # AW: AMBER
    3: TrafficLightState.GREEN,  # AW: GREEN

    99: TrafficLightState.INACTIVE,  # no AW state: represents a detected traffic light that is not active

    # additional states Autoware:
    # 18: UNKNOWN,
    # 4: WHITE,
    # additional states CommonRoad:
    # TrafficLightState.RED_YELLOW
    # TrafficLightState.INACTIVE
}


dict_autoware_to_commonroad_traffic_light_shape: Dict[int, TrafficLightDirection] = {
    5: TrafficLightDirection.ALL,  # AW: CIRCLE
    6: TrafficLightDirection.LEFT,  # AW: LEFT_ARROW
    7: TrafficLightDirection.RIGHT,  # AW: RIGHT_ARROW
    8: TrafficLightDirection.STRAIGHT,  # AW: UP_ARROW
    9: TrafficLightDirection.LEFT_STRAIGHT,  # AW: UP_LEFT_ARROW
    10: TrafficLightDirection.STRAIGHT_RIGHT,  # AW: UP_RIGHT_ARROW

    # additional states Autoware:
    # 11: DOWN_ARROW
    # 12: DOWN_LEFT_ARROW
    # 13: DOWN_RIGHT_ARROW
    # 18: UNKNOWN,
    # 0: CROSS,
    # additional states CommonRoad:
    # TrafficLightDirection.LEFT_RIGHT
}


dict_autoware_to_commonroad_traffic_light_status: Dict[int, bool] = {
    15: False,  # AW: SOLID_OFF
    16: True,  # AW: SOLID_ON

    # additional states Autoware:
    # 17: FLASHING
    # 18: UNKNOWN
}


def commonroad_shape_conversion(
        cr_object_type: ObstacleType,
        width: float,
        length: float,
        footprint: PolygonMsg,
        safety_margin: float = 0.0,
) -> Union[Rectangle, Circle, Polygon]:
    """
    Creates a CommonRoad shape from an CommonRoad obstacle type.

    :param cr_object_type: CommonRoad shape type of the obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    :param safety_margin: safety margin for the obstacle
    :return: CommonRoad shape
    """
    # add safety margin
    width = 2 * safety_margin + width
    length = 2 * safety_margin + length

    if cr_object_type in [ObstacleType.BUS, ObstacleType.TRUCK, ObstacleType.CAR, ObstacleType.BICYCLE, ObstacleType.MOTORCYCLE]:
        assert width > 0.0 and length > 0.0, "Obstacle shape: Width and length must be positive."
        return Rectangle(width=width, length=length)

    elif cr_object_type in [ObstacleType.PEDESTRIAN]:
        assert width > 0.0, "Obstacle shape: Width must be positive."
        return Circle(radius=(width / 2))

    elif cr_object_type in [ObstacleType.UNKNOWN]:
        # convert 3D polygon footprint to 2D
        points = footprint.points
        assert points, "Obstacle shape: Footprint must be provided."
        buffered_footprint = create_buffered_footprint(points, safety_margin)
        return Polygon(vertices=buffered_footprint)

    else:
        raise TypeError("Unsupported CommonRoad object type: " + str(cr_object_type))


def commonroad_shape_updater(
        dynamic_obstacle: DynamicObstacle,
        width: float,
        length: float,
        footprint: PolygonMsg,
        safety_margin: float,
) -> None:
    """
    Update the shape of a CommonRoad dynamic obstacle.

    :param dynamic_obstacle: CommonRoad dynamic obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    :param safety_margin: safety margin for the obstacle
    """
    shape = dynamic_obstacle.obstacle_shape
    # add safety margin
    width = 2 * safety_margin + width
    length = 2 * safety_margin + length

    if isinstance(shape, Rectangle):
        assert width > 0.0 and length > 0.0, "Update obstacle shape: Width and length must be positive."
        dynamic_obstacle.obstacle_shape.width = width
        dynamic_obstacle.obstacle_shape.length = length

    elif isinstance(shape, Circle):
        assert width > 0.0, "Update obstacle shape: Width must be positive."
        dynamic_obstacle.obstacle_shape.radius = width / 2

    elif isinstance(shape, Polygon):
        # convert 3D polygon footprint to 2D
        points = footprint.points
        assert points, "Update obstacle shape: Footprint must be provided."
        buffered_footprint = create_buffered_footprint(points, safety_margin)
        dynamic_obstacle.obstacle_shape.vertices = buffered_footprint

    else:
        raise TypeError("Unsupported CommonRoad shape type: " + str(dynamic_obstacle.obstacle_shape))


def create_buffered_footprint(
        points: List[Vector3],
        safety_margin: float
) -> np.ndarray:
    """
    Create a buffered footprint from a 3D polygon footprint.

    :param points: list of 3D points
    :param safety_margin: safety margin for the obstacle
    """
    footprint_2d = np.array([[point.x, point.y] for point in points])

    # add safety margin buffer for polygon
    polygon = PolygonShapely(footprint_2d)
    polygon_buffer = polygon.buffer(safety_margin, join_style="mitre")
    buffered_vertices = list(polygon_buffer.exterior.coords)
    buffered_footprint = np.array([[x, y] for x, y in buffered_vertices])
    return buffered_footprint


def convert_ros2_time_stamp_to_tuple(stamp) -> Tuple[int, int]:
    """
    Converts the time stamp of a message into a tuple[sec, nano_sec].

    :param stamp: ros2 stamp msg
    :return: stamp as Tuple[int,int]
    """
    seconds: int = stamp.sec
    nano_seconds: int = stamp.nanosec
    return (seconds, nano_seconds)


def convert_vector3_to_numpy(
        vector3: Vector3
) -> np.ndarray:
    """
    Transform position (in Autoware) to position (in CommonRoad).

    :param vector3: ros2 vector3 msg
    :return: numpy (3,) array
    """
    return np.array([vector3.x, vector3.y, vector3.z])


def convert_ros2_pose_to_cr_custom_state(
        pose: Pose,
        origin_transform: List[float],
        time_step: int,
        twist: Twist = None,
        stamp: Tuple[int,int] = None
) -> CustomState:
    """
    Converts ros2 pose into CommonRoad custom state object.

    :param pose: Autoware pose msg
    :param origin_transform: origin transformation from Autoware to CommonRoad
    :param time_step: CommonRoad time step
    :param twist: Autoware twist message
    :param stamp: ros2 time stamp
    :return: CommonRoad custom state object
    """
    translation_velocity_vector: List[float] = None
    velocity: float = None
    rotation_velocity_vector: List[float] = None
    yaw_rate: float = None
    ros2_time_stamp: Tuple[int,int] = None

    if(twist is not None):
        translation_velocity_vector: np.ndarray[float] = convert_vector3_to_numpy(twist.linear)
        rotation_velocity_vector: np.ndarray[float] = convert_vector3_to_numpy(twist.angular)
        velocity = math.sqrt(float(translation_velocity_vector[0])**2 + float(translation_velocity_vector[1]**2))
        yaw_rate: float = rotation_velocity_vector[2]

    if(stamp is not None):
        ros2_time_stamp = convert_ros2_time_stamp_to_tuple(stamp)

    position = transform_utils.map2utm(origin_transform, pose.position)
    orientation = transform_utils.quaternion2orientation(pose.orientation)
    return CustomState(position=position,
                       orientation=orientation,
                       velocity=velocity,
                       yaw_rate=yaw_rate,
                       translation_velocity_vector=translation_velocity_vector,
                       rotation_velocity_vector=rotation_velocity_vector,
                       time_step=time_step,
                       ros2_time_stamp=ros2_time_stamp)


def convert_ros2_trajectory_to_cr_trajectory(
        trajectory: List[TrajectoryPoint],
        origin_transform: List[float],
        stamp=None
) -> CRTrajectory:
    """
    Converts ros2 pose into CommonRoad custom state object.

    :param trajectory: list of Autoware trajectory points.
    :param origin_transform: origin transformation from Autoware to CommonRoad
    :param stamp: ros2 time stamp
    :return: CommonRoad trajectory
    """
    ros2_time_stamp: Tuple[int,int] = None
    if(stamp is not None):
        ros2_time_stamp = convert_ros2_time_stamp_to_tuple(stamp)

    state_list: List[CustomState] = list()
    for i in range(len(trajectory)):
        # compute new position
        trajectory_point: TrajectoryPoint = trajectory[i]
        position = transform_utils.map2utm(origin_transform, trajectory_point.pose.position)
        orientation = transform_utils.quaternion2orientation(trajectory_point.pose.orientation)
        lon_velocity = trajectory_point.longitudinal_velocity_mps
        new_state = CustomState(position=position,
                                orientation=orientation,
                                velocity=lon_velocity,
                                time_step=i,
                                ros2_time_stamp=ros2_time_stamp)
        state_list.append(new_state)

    if(len(state_list) == 0):
        raise ValueError('trajectory state list is empty')

    return CRTrajectory(0, state_list)


def convert_ros2_predicted_path_to_cr_trajectory(
        predicted_path: PredictedPath,
        origin_transform: List[float]
) -> CRTrajectory:
    """
    Converts ros2 pose into Commonroad custom state object.

    :param predicted_path: predicted Autoware path
    :param origin_transform: origin transformation from Autoware to CommonRoad
    :return: CommonRoad trajectory object
    """
    state_list: List[CustomState] = list()
    for i, pose in enumerate(predicted_path.path):
        # compute new position
        position = transform_utils.map2utm(origin_transform,pose.position)
        orientation = transform_utils.quaternion2orientation(pose.orientation)
        new_state = CustomState(position=position,
                                orientation=orientation,
                                velocity=0,
                                time_step=i)
        state_list.append(new_state)

    if(len(state_list) == 0):
        raise ValueError('predicted path conversion: state list is empty')

    return CRTrajectory(0, state_list)


def convert_ros2_predicted_objects_to_cr_dynamic_obstacles(
        msg: PredictedObjects,
        origin_transform: List[float],
        step: int,
        stamp=None
) -> List[DynamicObstacle]:
    """
    Converts a ROS2 detected object to a CommonRoad static obstacle.

    :param msg: Autoware predicted objects message
    :param origin_transform: origin transformation from Autoware to CommonRoad map
    :param step: time step
    :param stamp: ros2 time stamp
    :return: list of CommonRoad dynamic obstacles of time-step
    """
    if(stamp is not None):
        ros2_time_stamp: Tuple[int,int] = convert_ros2_time_stamp_to_tuple(stamp)

    # PredictedObjects.objects = list[PredictedObject]
    obstacle_list: List[DynamicObstacle] = list()

    for idx, predicted_object in enumerate(msg.objects):
        uuid: np.ndarray = predicted_object.object_id.uuid
        time_step: int = step

        # WARNING: This value does not make sense in tum
        existance_probability: float = predicted_object.existence_probability

        # Obstacle type
        obstacle_type: ObstacleType = (
            dict_autoware_to_cr_obstacle_type[
                get_classification_with_highest_probability(predicted_object.classification)]
        )

        # Convert shape
        shape: Union[Rectangle, Circle, Polygon] = commonroad_shape_conversion(
            cr_object_type=obstacle_type,
            width=predicted_object.shape.dimensions.y,
            length=predicted_object.shape.dimensions.x,
            footprint=predicted_object.shape.footprint
        )


        # obstacle state
        kinematics = predicted_object.kinematics
        pose: Pose = kinematics.initial_pose_with_covariance.pose
        twist: Twist = kinematics.initial_twist_with_covariance.twist
        cr_state: CustomState = convert_ros2_pose_to_cr_custom_state(pose,
                                                                    origin_transform,
                                                                    time_step,
                                                                    twist=twist)
        initial_state = InitialState(
            position=cr_state.position,
            orientation=cr_state.orientation,
            velocity=cr_state.velocity,
            acceleration=0.0,
            yaw_rate=cr_state.yaw_rate,
            slip_angle=0.0,
            time_step=time_step,
        )

        # predicted trajectory
        predicted_trajectory: CRTrajectory = get_cr_trajectory_with_with_highest_probability(kinematics.predicted_paths,
                                                                                        origin_transform)
        trajectory_prediction = TrajectoryPrediction(
            trajectory=predicted_trajectory,
            shape=shape
        )

        # obstacle generation
        dynamic_obstacle = DynamicObstacle(
            obstacle_id=convert_uuid_to_int(uuid),
            obstacle_type=obstacle_type,
            obstacle_shape=shape,
            initial_state=initial_state,
            prediction=trajectory_prediction,
            ros2_time_stamp=ros2_time_stamp,
            uuid=uuid
        )

        obstacle_list.append(dynamic_obstacle)

    return obstacle_list


def convert_traffic_lights(
        msg: TrafficSignalArray,
        stamp
) -> Tuple[Tuple[int, int], List[TrafficLight]]:
    """
    Converts Autoware traffic lights to CommonRoad at time step.

    !Warning!: Due to missing kwargs CR TrafficLight

    :param msg: TrafficSignalArray Autoware msg
    :param stamp: ros2 time stamp
    :return: Tuple[ros2_time_stamp, List[TrafficLight_At_Time_stamp]]
    """
    ros2_time_stamp: Tuple[int, int] = convert_ros2_time_stamp_to_tuple(stamp)

    # TrafficSignalArray.signals: List[TrafficSignal] and TrafficSignal.elements: List[TrafficSignalElements]
    traffic_light_list: List[TrafficLight] = list()

    # Create a dummy traffic light as position [0, 0] with correct id so the values can be reassigned afterwards
    for traffic_signal in msg.signals:
        highest_confidence_light = max(traffic_signal.lights, key=lambda x: x.confidence)
        traffic_light_list.append(
            TrafficLight(
                traffic_light_id=traffic_signal.map_primitive_id,
                position=np.asarray([0, 0]),
                color=[dict_autoware_to_commonroad_traffic_light_color[highest_confidence_light.color]],
                active=dict_autoware_to_commonroad_traffic_light_status[highest_confidence_light.status],
                direction=highest_confidence_light.shape
            )
        )

    return (ros2_time_stamp, traffic_light_list)


def convert_uuid_to_int(
        uuid: np.ndarray,
        offset: int = 200000
) -> int:
    """
    Quick and dirty conversion for uuid.

    :param uuid: array of int
    :param offset: offset so no clash with other cr-ids occurs
    :return:
    """
    id_str: str = ""
    for idx in range(uuid.shape[0]):
        id_str += str(uuid[idx])

    return int(id_str) + offset


def get_classification_with_highest_probability(
        classification: List[ObjectClassification]
) -> int:
    """
    Finds class with the highest probability.

    :param classification: list of classification objects
    :return: label (int) of highest classification
    """
    return sorted(classification, key=lambda x: x.probability, reverse=True)[0].label


def get_cr_trajectory_with_with_highest_probability(
                path_list: List[PredictedPath],
                origin_transform: List[float]
) -> CRTrajectory:
    """
    Finds predicted path with the highest confidence and returns a CRTrajectory from it.

    :param path_list: list of Autoware predicted path
    :param origin_transform: origin transformation from Autoware to CommonRoad
    :return: CommonRoad trajectory object
    """
    best_path: PredictedPath = sorted(path_list, key=lambda x: x.confidence, reverse=True)[0]
    return convert_ros2_predicted_path_to_cr_trajectory(best_path, origin_transform)


def save_cr_xml_scenario(
        scenario: Scenario,
        save_path: str,
        filename: str,
        planning_problem: PlanningProblem=None
) -> None:
    """
    Store converted map as CommonRoad scenario.

    :param save_path: root dir path to save the CommonRoad xml to.
    :param filename: filename of the CommonRoad scenario
    :param planning_problem: CommonRoad planning problem
    """
    planning_problem_set = PlanningProblemSet()
    if (planning_problem is not None):
        planning_problem_set.add_planning_problem(planning_problem)

    writer = CommonRoadFileWriter(
        scenario=scenario,
        planning_problem_set=planning_problem_set,
        author="Edgar Data Generation",
        affiliation="Technical University of Munich",
        source="edgar drives"
    )
    os.makedirs(save_path, exist_ok=True)
    writer.write_to_file(
        os.path.join(save_path, "".join([filename, ".xml"])),
        OverwriteExistingFile.ALWAYS,
    )


def set_traffic_light_cycle(traffic_light_state: TrafficLightState) -> TrafficLightCycle:
    """
    Set the traffic light cycle based on the traffic light state.

    :param traffic_light_state: traffic light state
    :return: traffic light cycle
    """

    cycle_element = TrafficLightCycleElement(traffic_light_state, 5)

    traffic_light_cylce = TrafficLightCycle(cycle_elements=[cycle_element])

    return traffic_light_cylce


def uuid_from_ros_msg(
        msg: UUIDMsg
) -> PyUUID:
    """
    Converts a ROS UUID message to a Python type UUID.

    ROS UUID is represented as uint8[16], i.e., an array of length 16 containing 8-byte unsigned integers.

    :param msg: ROS UUID message of type unique_identifier_msgs/msg/UUID
    :return:
    """
    return PyUUID(bytes=bytes([int(_elem) for _elem in msg]))
