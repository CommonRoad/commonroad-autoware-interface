# standard imports
import math
import os


# third party imports
import numpy as np

# ROS imports
import rclpy.logging as ros_logging

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
from commonroad.geometry.shape import Shape, Rectangle, Circle, Polygon


# Third party
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_planning_msgs.msg import TrajectoryPoint  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects, ObjectClassification,PredictedPath  # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from geometry_msgs.msg import Pose # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
from geometry_msgs.msg import Polygon as PolygonMsg # type: ignore


# own code base
import cr2autoware.common.utils.transform as transform_utils

# typing
from typing import List, Dict, Tuple, Union


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



def convert_ros2_time_stamp_to_tuple(stamp) -> Tuple[int, int]:
    """
    Converts the time stamp of a message into a tuple[sec, nano_sec]
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
    Transform position (in autoware) to position (in commonroad).
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
    Converts ros2 pose into commonroad custom state object:
    :param pose: Autoware pose msg
    :param origin_transform:origin transformation from autoware to CommonRoad
    :param time_step: commonroad time step
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
    Converts ros2 pose into commonroad custom state object.
    :param trajectory: list of autoware trajectory points.
    :param origin_transform: origin transformation from autoware to CommonRoad
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
    Converts ros2 pose into commonroad custom state object.
    :param predicted_path: predicted autoware path
    :param origin_transform: origin transformation from autoware to CommonRoad
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
)->List[DynamicObstacle]:
    """
    Converts a ROS2 detected object to a CommonRoad static obstacle.
    :param msg: Autoware predicted objects message
    :param origin_transform: origin transformation from autoware to CommonRoad map
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


        # Convert shape
        shape: Union[Rectangle, Circle, Polygon] = convert_aw_shape_to_cr_shape(
            aw_shape_type=predicted_object.shape,
            width=predicted_object.shape.dimensions.x,
            length=predicted_object.shape.dimensions.y,
            footprint=predicted_object.shape.footprint
        )


        # Obstacle type
        obstacle_type: ObstacleType = (
            dict_autoware_to_cr_obstacle_type[
                get_classification_with_highest_probability(predicted_object.classification)]
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


def convert_uuid_to_int(
        uuid: np.ndarray,
        offset: int = 200000
) -> int:
    """
    Quick and dirty conversion for uuid.
    :param uuid: array of int
    :param offset: offset so no clash with other cr-ids occurs
    :return: integer
    """
    id_str: str = ""
    for el in uuid.shape[0]:
        id_str += str(el)

    return int(id_str) + offset



def get_classification_with_highest_probability(
        classification: List[ObjectClassification]
) -> int:
    """
    Finds class with highest probatility.
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
    :param path_list: list of Autoware pridictited path
    :param origin_transform: origin transformation from autoware to CommonRoad
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


def convert_aw_shape_to_cr_shape(
        aw_shape_type: int,
        width: float,
        length: float,
        footprint: PolygonMsg,
) -> Union[Rectangle, Circle, Polygon]:
    """Convert Autoware shape to CommonRoad shape.

    :param aw_shape_type: Autoware shape type of the obstacle
    :param width: width of the obstacle
    :param length: length of the obstacle
    :param footprint: a specification of a polygon
    :return: CommonRoad shape
    """
    if aw_shape_type == 0:
        assert width > 0.0 and length > 0.0, "Obstacle shape: Width and length must be positive."
        return Rectangle(width=width, length=length)

    elif aw_shape_type == 1:
        assert width > 0.0, "Obstacle shape: Width must be positive."
        return Circle(radius=(width / 2))

    elif aw_shape_type == 2:
        # convert 3D polygon footprint to 2D
        points = footprint.points
        assert points, "Obstacle shape: Footprint must be provided."
        footprint_2d = np.array([[point.x, point.y] for point in points])
        return Polygon(vertices=footprint_2d)

    else:
        raise TypeError("Unsupported Autoware shape type: " + str(aw_shape_type))
