import os
import argparse

import pickle

from rclpy.serialization import deserialize_message # type: ignore
from rosidl_runtime_py.utilities import get_message  # type: ignore
from std_msgs.msg import String  # type: ignore
import rosbag2_py  # type: ignore

# Third party
from autoware_auto_perception_msgs.msg import PredictedObjects  # type: ignore
from std_msgs.msg import Header # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from geometry_msgs.msg import Pose # type: ignore
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects, PredictedObject  # type: ignore
from autoware_auto_perception_msgs.msg import TrafficSignalArray # type: ignore



# commonroad
from commonroad.scenario.state import CustomState
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.scenario.traffic_light import TrafficLight


# own code base
import cr2autoware.common.utils.cr_conversion_utils as cr_conv_utils


# typing
from typing import List, Any, Tuple




class Rosbag2CR_Converter:
    """
    Reads in mcaps, converts the content to CommonRoad and saves the objects as pkl data, so that further
    processing is independent of Ros2 and Autoware.

    ========= Topics
    "/initialpose3d"
    "/planning/mission_planning/goal"
    "/planning/scenario_planning/trajectory_smoothed"
    "/planning/scenario_planning/trajectory"
    "/localization/kinematic_state"
    "/perception/object_recognition/objects"
    "/perception/traffic_light_recognition/traffic_signals"
    """

    def __init__(
            self,
            path_to_rosbag: str,
            path_to_origin_transform: str,
            saving_path_pkl: str,
            scenario_identifier: str
    ) -> None:
        """
        Converts rosbag to commonroad pkl.
        :param path_to_rosbag: path to recorded mcap rosbag
        :param path_to_origin_transform: path to saved origin transform from autoware to commonroad map
        :param saving_path_pkl: where to save the pickles as abspath
        :param scenario_identifier: identifier of the scenario
        """
        self._path_to_rosbag: str = path_to_rosbag
        self._saving_path_pkl: str = saving_path_pkl
        self._path_to_origin_transform: str = path_to_origin_transform
        self._scenario_identifier: str = scenario_identifier
        self._sanity_check_paths()

        # Load origin transform
        self.origin_transform: List[float] = None
        self._load_origin_transform()

        # Init _reader
        self._reader = None
        self._init_reader()

        # Helper for trajectory data
        self._odometry_time_step: int = 0
        self._perception_time_step: int = 0
        self._odometry_data: List[Odometry] = list()

        # converted data
        self._cr_initial_pose3d_data: List[CustomState] = list()
        self._cr_goal_pose_data: List[CustomState] = list()
        self._cr_reference_trajectory_data: List[CRTrajectory] = list()
        self._cr_planned_trajectory_data: List[CRTrajectory] = list()
        self._cr_driven_trajectory_data: List[CRTrajectory] = list()
        self._cr_predicted_objects_data: List[List[DynamicObstacle]] = list()
        self._cr_traffic_light_data: List[Tuple[Tuple[int, int], List[TrafficLight]]] = list()


    def read_rosbag(self) -> None:
        """
        Reads out rosbags and converts it to commonroad data
        """
        topic_types = self._reader.get_all_topics_and_types()

        # Read out messages and convert
        while self._reader.has_next():
            topic, data, timestamp = self._reader.read_next()
            msg_type = get_message(self._typename(topic, topic_types))
            msg = deserialize_message(data, msg_type)

            print(f"reading in topic {topic}")

            if(topic == "/initialpose3d"):
                self._convert_initialpose3d(msg)

            elif(topic == "/planning/mission_planning/goal"):
                self._convert_goal_pose(msg)

            elif(topic == "/planning/scenario_planning/trajectory_smoothed"):
                self._convert_route_trajectory(msg)

            elif(topic == "/planning/scenario_planning/trajectory"):
                self._convert_planned_trajectory(msg)

            elif(topic == "/localization/kinematic_state"):
                self._convert_odometry_msg(msg)

            elif(topic == "/perception/object_recognition/objects"):
                self._convert_predicted_objects(msg)

            elif(topic == "/perception/traffic_light_recognition/traffic_signals"):
                self._convert_traffic_lights(msg)

            else:
                raise ValueError(f"Topic with name {topic} could not be assigned to known topics")

        # At the end generate driven trajectory
        self._generate_driven_trajectory()




    def save_cr_data_to_pkl(self) -> None:
        """
        Save data to pkl files.
        """

        # generate dir for current data
        fileformat: str = ".pkl"
        current_dir_name = self._saving_path_pkl + "/" + self._scenario_identifier
        os.mkdir(current_dir_name)

        # Construct names
        initial_pose3d_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                            data_identifier="initial_pose3d",
                                                            scenario_identifier=self._scenario_identifier,
                                                            fileformat=fileformat)

        goal_pose_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                       data_identifier="goal_pose",
                                                       scenario_identifier=self._scenario_identifier,
                                                       fileformat=fileformat)

        reference_trajectory_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                                  data_identifier="reference_trajectory",
                                                                  scenario_identifier=self._scenario_identifier,
                                                                  fileformat=fileformat)


        planned_trajectory_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                                data_identifier="planned_trajectory",
                                                                scenario_identifier=self._scenario_identifier,
                                                                fileformat=fileformat)

        driven_trajectory_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                               data_identifier="driven_trajectory",
                                                               scenario_identifier=self._scenario_identifier,
                                                               fileformat=fileformat)

        predicted_obstacles_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                                 data_identifier="predicted_obstacles",
                                                                 scenario_identifier=self._scenario_identifier,
                                                                 fileformat=fileformat)


        traffic_lights_file: str = self._construct_pkl_name(current_dir_name=current_dir_name,
                                                                 data_identifier="traffic_lights",
                                                                 scenario_identifier=self._scenario_identifier,
                                                                 fileformat=fileformat)


        # write to pickle files
        with open(initial_pose3d_file, 'wb') as f:
            pickle.dump(self._cr_initial_pose3d_data, f)

        with open(goal_pose_file, 'wb') as f:
            pickle.dump(self._cr_goal_pose_data, f)

        with open(reference_trajectory_file, 'wb') as f:
            pickle.dump(self._cr_reference_trajectory_data, f)

        with open(planned_trajectory_file, 'wb') as f:
            pickle.dump(self._cr_planned_trajectory_data, f)

        with open(driven_trajectory_file, 'wb') as f:
            pickle.dump(self._cr_driven_trajectory_data, f)

        with open(predicted_obstacles_file, 'wb') as f:
            pickle.dump(self._cr_predicted_objects_data, f)

        with open(traffic_lights_file, 'wb') as f:
            pickle.dump(self._cr_traffic_light_data, f)




    @staticmethod
    def _typename(
            topic_name: Any,
            topic_types: Any
    ) -> Any:
        """
        Get topic type
        :param topic_name: name of topic
        :param topic_types: types of all topics
        :return: topic type
        """
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")


    def _init_reader(self) -> None:
        """
        Init rosbag _reader
        """
        self._reader = rosbag2_py.SequentialReader()
        self._reader.open(
            rosbag2_py.StorageOptions(uri=self._path_to_rosbag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

    def _load_origin_transform(self) -> None:
        """
        Load origin transform
        """
        with open(self._path_to_origin_transform, 'rb') as f:
            self.origin_transform = pickle.load(f)


    def _sanity_check_paths(self) -> None:
        """
        Checks whether paths exist
        """
        if (not os.path.exists(self._path_to_rosbag)):
            raise FileNotFoundError(f"Could not find rosbag path {self._path_to_rosbag}")

        if(not os.path.exists(self._saving_path_pkl)):
            raise FileNotFoundError(f"Could not find path to save pkls to: {self._saving_path_pkl}")

        if(not os.path.exists(self._path_to_origin_transform)):
            raise FileNotFoundError(f"Could not find origin transform pickle at: {self._path_to_origin_transform}")


    def _construct_pkl_name(
            self,
            current_dir_name: str,
            data_identifier: str,
            scenario_identifier: str,
            fileformat: str='.pkl'
    ) -> str:
        """
        Constructs pkl names from dir name and identifiers
        :param current_dir_name: root dir for data saving
        :param data_identifier: identifies data (e.g. kinematic state)
        :param scenario_identifier: identifies scenario (e.g. Bakery Lab)
        :param fileformat: saving file format
        :return: string for saving
        """
        return current_dir_name + "/" + data_identifier + "_" + scenario_identifier + fileformat




    ################################################# ROS2 Conversion #################################################

    def _convert_initialpose3d(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Converts initial pose to commonroad.
        :param msg: initial pose msg
        """
        pose: Pose = msg.pose.pose
        stamp = msg.header.stamp
        cr_state: CustomState = cr_conv_utils.convert_ros2_pose_to_cr_custom_state(
            pose,
            self.origin_transform,
            0,
            stamp=stamp
        )
        self._cr_initial_pose3d_data.append(cr_state)


    def _convert_goal_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Converts goal pose msg to commonroad.
        :param msg: goal pose msg.
        """
        # goal pose
        pose: Pose = msg.pose
        stamp = msg.header.stamp
        cr_state: CustomState = cr_conv_utils.convert_ros2_pose_to_cr_custom_state(
            pose,
            self.origin_transform,
            len(self._cr_reference_trajectory_data) - 1,
            stamp=stamp
        )
        self._cr_goal_pose_data.append(cr_state)


    def _convert_route_trajectory(self, msg: AWTrajectory) -> None:
        """
        Converts route with velocity profile.
        :param msg: reference path msg
        """
        stamp = msg.header.stamp
        cr_trajectory: CRTrajectory = cr_conv_utils.convert_ros2_trajectory_to_cr_trajectory(
            msg.points,
            self.origin_transform,
            stamp=stamp
        )
        self._cr_reference_trajectory_data.append(cr_trajectory)



    def _convert_planned_trajectory(self, msg: AWTrajectory) -> None:
        """
        Converts planned trajectory.
        :param msg: planned trajecoty msg
        """
        stamp = msg.header.stamp
        cr_trajectory: CRTrajectory = cr_conv_utils.convert_ros2_trajectory_to_cr_trajectory(
            msg.points,
            self.origin_transform,
            stamp=stamp
        )
        self._cr_planned_trajectory_data.append(cr_trajectory)



    def _convert_predicted_objects(self, msg: PredictedObjects):
        """
        Converts predicted objects.
        :param msg: predicted objects msg.
        """
        # objects
        stamp = msg.header.stamp
        dynamic_obstacles_at_time_step: List[DynamicObstacle] = (
            cr_conv_utils.convert_ros2_predicted_objects_to_cr_dynamic_obstacles(
                msg=msg,
                origin_transform=self.origin_transform,
                step=self._perception_time_step,
                stamp=stamp
            )
        )
        self._cr_predicted_objects_data.append(dynamic_obstacles_at_time_step)
        self._perception_time_step += 1


    def _convert_odometry_msg(self, msg: Odometry) -> None:
        """
        Converts driven trajectory.
        :param msg: kinematic state message
        """
        stamp = msg.header.stamp
        pose: Pose = msg.pose.pose
        twist: Twist = msg.twist.twist
        cr_state: CustomState = cr_conv_utils.convert_ros2_pose_to_cr_custom_state(
            pose,
            self.origin_transform,
            self._odometry_time_step,
            twist=twist,
            stamp=stamp
        )
        self._odometry_data.append(cr_state)
        self._odometry_time_step += 1


    def _generate_driven_trajectory(self) -> None:
        """
        Generates cr trajectory data for ego vehicle after all messages have been read out
        """
        self._cr_driven_trajectory_data.append(CRTrajectory(0, self._odometry_data))


    def _convert_traffic_lights(self, msg: TrafficSignalArray) -> None:
        """
        Converts traffic lights per step to CommonRoad.
        """
        stamp = msg.header.stamp
        # Tuple[Tuple[int, int], List[TrafficLight]] = (ros2_time_stamp, list_of_traffic_lights_at_time_stamp)
        traffic_light_tuple_at_stamp: Tuple[Tuple[int, int], List[TrafficLight]] = cr_conv_utils.convert_traffic_lights(
            msg=msg,
            stamp=stamp
        )
        self._cr_traffic_light_data.append(traffic_light_tuple_at_stamp)




if __name__ == "__main__":
    # CLI parser
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--rosbag", "-r", help="abspath as str to rosbag")
    parser.add_argument("--transform", "-t", help="abspath as str to pkl of origin transform")
    parser.add_argument("--identifier", "-i", help="name that the scenario should have")
    parser.add_argument("--pkl", "-p", help="abspath as str to where pickle root folder for saving is")

    args = vars(parser.parse_args())

    # Converter methods
    converter = Rosbag2CR_Converter(
        path_to_rosbag=args["rosbag"],
        path_to_origin_transform=args["transform"],
        saving_path_pkl=args["pkl"],
        scenario_identifier=args["identifier"]
    )

    converter.read_rosbag()
    converter.save_cr_data_to_pkl()
