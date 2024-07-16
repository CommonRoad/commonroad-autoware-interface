# standard imports
import os
from enum import Enum
from typing import List
from typing import TYPE_CHECKING
from typing import Any
import datetime

# third party imports
import pickle

# ROS imports
from rclpy.serialization import serialize_message, deserialize_message # type: ignore
from rosidl_runtime_py.utilities import get_message # type: ignore
import rosbag2_py # type: ignore

# ROS msgs
from std_msgs.msg import Header # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from geometry_msgs.msg import Pose # type: ignore
from std_msgs.msg import String # type: ignore

# Autoware msgs
from autoware_auto_perception_msgs.msg import PredictedObjects  # type: ignore
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects, PredictedObject  # type: ignore
from autoware_auto_perception_msgs.msg import TrafficSignalArray  # type: ignore

# cr2autoware
from cr2autoware.handlers.base import BaseHandler
from cr2autoware.common.ros_interface.create import create_subscription
from cr2autoware.common.ros_interface.specs_subscriptions import \
    spec_initial_pose_sub, spec_echo_back_goal_pose_sub, spec_traj_smoothed, spec_traj, \
    spec_objects_sub, spec_odometry, spec_traffic_lights

# type checking
if TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


class SaverCommands(Enum):
    """
    Commands for the saver class
    """
    RUN = 1
    STOP = 0


class DataGenerationHandler(BaseHandler):
    """
    Saves the relevant topics as mcap rosbags.

    -------------------
    **Subscribers:**

    * _initial_pose3d_subscriber:
        * Description: Initial pose of the vehicle
        * Topic: `/initialpose3d`
        * Message Type: `geometry_msgs.msg.PoseWithCovarianceStamped`
    * _goal_pose_subscriber:
        * Description: Goal pose of the vehicle
        * Topic: `/planning/mission_planning/goal`
        * Message Type: `geometry_msgs.msg.PoseStamped`
    * _reference_trajectory_subscriber:
        * Description: Reference trajectory with smoothed velocity profile
        * Topic: `/planning/scenario_planning/trajectory_smoothed`
        * Message Type: `autoware_auto_planning_msgs.msg.Trajectory`
    * _planned_trajectory_subscriber:
        * Description: Planned trajectory
        * Topic: `/planning/scenario_planning/trajectory`
        * Message Type: `autoware_auto_planning_msgs.msg.Trajectory`
    * _driven_trajectory_subscriber:
        * Description: Driven trajectory from localization
        * Topic: `/localization/kinematic_state`
        * Message Type: `nav_msgs.msg.Odometry`
    * _predicted_objects_subscriber:
        * Description: Predicted objects
        * Topic: `/perception/object_recognition/objects`
        * Message Type: `autoware_auto_perception_msgs.msg.PredictedObjects`
    * _traffic_lights_subscriber:
        * Description: Traffic lights
        * Topic: `/perception/traffic_light_recognition/traffic_signals`
        * Message Type: `autoware_auto_perception_msgs.msg.TrafficSignalArray`

    -------------------
    :var _command_status: command status for the saver
    :var _save_path: absolute path where the data should be saved to
    :var _save_id: id string for the origin transform and the mcap file
    :var _writer: rosbag writer
    :var _initial_pose3d_subscriber: subscriber for initial pose
    :var _goal_pose_subscriber: subscriber for goal pose
    :var _reference_trajectory_subscriber: subscriber for reference trajectory
    :var _planned_trajectory_subscriber: subscriber for planned trajectory
    :var _driven_trajectory_subscriber: subscriber for driven trajectory
    :var _detected_objects_subscriber: subscriber for detected objects
    :var _predicted_objects_subscriber: subscriber for predicted objects
    :var _traffic_lights_subscriber: subscriber for traffic lights
    :var _origin_transformation: coordinate transformation between autoware and commonroad
    """
    def __init__(self,
                 cr2aw_node: "Cr2Auto",
                 save_path: str,
                 save_id: str,
                 origin_transformation: List[float]
                 ) -> None:
        """
        Constructor for DataGenerationHandler class.

        :param cr2aw_node: ros2 node for cr2autoware interface
        :param save_path: absolute path where the data should be saved to.
        :param save_id: id string for the origin transform and the mcap file
        :param origin_transformation: coordinate transformation between autoware and commonroad
        """

        super().__init__(node=cr2aw_node,
                         logger=cr2aw_node.get_logger().get_child("data_generation_handler"),
                         verbose=cr2aw_node.verbose)


        # command status -> 0:stop, 1:start+delete_old_data,
        self._command_status = SaverCommands.STOP

        # saving
        if (not os.path.exists(save_path)):
            self._logger.warning(f"_save_path not found: {save_path}; trying: /home/edgar/av20/team_software/data_saving")
            save_path = "/home/edgar/av20/team_software/data_saving"
            if (not os.path.exists(save_path)):
                self._logger.error(f"_save_path not found: {save_path}; CANCELING NODE!")
                raise FileNotFoundError(
                    f"ERROR: _save_path for CommonRoad saving not found: {save_path}; CANCELING NODE!"
                    f"_save_path is currently map_path/datasaving ")

        self._save_path: str = save_path
        self._save_id: str = save_id


        self._logger.info(f"Delete old recording")
        self._logger.info(f"Start recording data")

        # rosbag _writer
        self._writer = None
        self._init_writer()

        # init subscriptions
        self._initial_pose3d_subscriber = None
        self._goal_pose_subscriber = None
        self._reference_trajectory_subscriber = None
        self._planned_trajectory_subscriber = None
        self._driven_trajectory_subscriber = None
        self._detected_objects_subscriber = None
        self._predicted_objects_subscriber = None
        self._traffic_lights_subscriber = None
        self._init_subscriptions()

        # Save origin transform to pkl
        self.origin_transformation: List[float] = None
        self.origin_transformation = origin_transformation
        self._save_origin_transformation_to_pkl()



    @property
    def command_status(self) -> SaverCommands:
        """
        Property for command status.

        :return: current command status
        """
        return self._command_status


    @property
    def node(self) -> "Cr2Auto":
        """
        Property for the cr2autoware node.

        :return: the cr2autoware node
        """
        return self._node


    @property
    def save_path(self) -> str:
        """
        Property for the save path.

        :return: save path
        """
        return self._save_path


    @save_path.setter
    def save_path(self, save_path: str) -> None:
        """
        :param save_path: new absolute path where saving should take place
        """
        if (not os.path.exists(save_path)):
            self._logger.error(f"_save_path not found: {save_path}")
            raise FileNotFoundError(f'_save_path not found: {save_path}')

        self._save_path = save_path




    def start_recording(self) -> None:
        """Starts recording topics that would otherwise be spammed constantly."""
        if (self._command_status is not SaverCommands.STOP):
            return

        self._command_status = SaverCommands.RUN


    def stop_recording_and_save_data(self) -> None:
        """Stops recording and saves data."""
        self._logger.info(f"Stopped recording data!")
        self._command_status = SaverCommands.STOP
        del self._writer
        # init writer again for next recording with new save_id
        self._save_id = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self._init_writer()
        self._logger.info(f"Initialized new writer for next recording!")
        self.start_recording()
        self._save_origin_transformation_to_pkl()

    def _add_time_header_to_msg(self, msg: Any) -> Any:
        """
        Adds a header an autoware msg.

        :param msg: msg to add header to
        """
        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()
        msg.header = header
        return msg


    def _save_origin_transformation_to_pkl(self) -> None:
        """Saves origin transform to pkl."""
        pkl_name: str = self.save_path + "/" + self._save_id + "_origin_transform.pkl"
        with open(pkl_name, 'wb') as f:
            pickle.dump(self.origin_transformation, f)




    ####################### Implement not needed abstract methods ####################
    def _init_parameters(self) -> None:
        """Overides Abstact: Retrieve required ROS params from self._node."""
        # Does not need params
        pass


    def _init_publishers(self) -> None:
        """Overides AbstractInitialize required publishers for self._node."""
        # Does not have a publisher
        pass






    ###################################--- ROS2 Related --- #####################################################
    def _init_writer(self) -> None:
        """Creates topic writers."""
        # Rosbag _writer
        self._writer = rosbag2_py.SequentialWriter()

        self._writer.open(
            rosbag2_py._storage.StorageOptions(
                uri=self._save_path + "/" + self._save_id + "_output.mcap",
                storage_id="mcap"
            ),
            rosbag2_py._storage.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"
            ),
        )

        # Goal pose
        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/initialpose3d",
                type="geometry_msgs/msg/PoseWithCovarianceStamped",
                serialization_format="cdr"
            )
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/planning/mission_planning/goal",
                type="geometry_msgs/msg/PoseStamped",
                serialization_format="cdr"
            )
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/planning/scenario_planning/trajectory_smoothed",
                type="autoware_auto_planning_msgs/msg/Trajectory",
                serialization_format="cdr"
            )
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/planning/scenario_planning/trajectory",
                type="autoware_auto_planning_msgs/msg/Trajectory",
                serialization_format="cdr"
            )
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/localization/kinematic_state",
                type="nav_msgs/msg/Odometry",
                serialization_format="cdr"
            )
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/perception/object_recognition/objects",
                type="autoware_auto_perception_msgs/msg/PredictedObjects",
                serialization_format="cdr"
            )
        )

        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/perception/traffic_light_recognition/traffic_signals",
                type="autoware_auto_perception_msgs/msg/TrafficSignalArray",
                serialization_format="cdr"
            )
        )

    def _init_subscriptions(self) -> None:
        """Creates subscribers to relevant topics."""

        # Initial pose
        self._initial_pose3d_subscriber = create_subscription(
            self._node,
            spec_initial_pose_sub,
            self.initial_pose3d_callback,
            self._node.callback_group,
        )

        # Goal pose
        self._goal_pose_subscriber = create_subscription(
            self._node,
            spec_echo_back_goal_pose_sub,
            self.goal_pose_callback,
            self._node.callback_group,
        )

        # Reference path with velocity profile
        self._reference_trajectory_subscriber = create_subscription(
            self._node,
            spec_traj_smoothed,
            self.reference_trajectory_callback,
            self._node.callback_group
        )

        # planned trajectory
        self._planned_trajectory_subscriber = create_subscription(
            self._node,
            spec_traj,
            self.planned_trajectory_callback,
            self._node.callback_group
        )

        # driven trajectory from localization
        self._driven_trajectory_subscriber = create_subscription(
            self._node,
            spec_odometry,
            self.driven_trajectory_callback,
            self._node.callback_group
        )

        # object perception and prediction
        self._predicted_objects_subscriber = create_subscription(
            self._node,
            spec_objects_sub,
            self.predicted_objects_callback,
            self._node.callback_group
        )

        # traffic lights
        self._traffic_lights_subscriber = create_subscription(
            self.node,
            spec_traffic_lights,
            self.traffic_lights_callback,
            self._node.callback_group
        )


    # callbacks
    # ----------
    def initial_pose3d_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Callback to initial pose data.

        :param msg: msg for 3d initial pose
        """
        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/initialpose3d",
            serialize_message(msg),
            1
        )


    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback to gather goal pose data.

        :param msg: goal pose msg
        """
        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/planning/mission_planning/goal",
            serialize_message(msg),
            1
        )
    def reference_trajectory_callback(self, msg: AWTrajectory) -> None:
        """
        Callback to gather reference trajectory data.

        :param msg: message reference trajectory with smoothed velocity
        """
        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/planning/scenario_planning/trajectory_smoothed",
            serialize_message(msg),
            1
        )

    def planned_trajectory_callback(self, msg: AWTrajectory) -> None:
        """
        Callback to gather reference trajectory data.

        :param msg: planned trajectory msg
        """
        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/planning/scenario_planning/trajectory",
            serialize_message(msg),
            1
        )

    def driven_trajectory_callback(self, msg) -> None:
        """
        Callback to gather driven trajectory kinematic data.

        :param msg: msg for current kinematic state
        """
        if(self._command_status != SaverCommands.RUN):
            return

        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/localization/kinematic_state",
            serialize_message(msg),
            1
        )

    def predicted_objects_callback(self, msg: PredictedObjects) -> None:
        """
        Callback to gather obstacle data from prediciton.

        :param msg: msg for predicted objects
        """
        if (self._command_status != SaverCommands.RUN):
            return

        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/perception/object_recognition/objects",
            serialize_message(msg),
            1
        )

    def traffic_lights_callback(self, msg) -> None:
        """
        Callback to gather traffic light status.

        :param msg: msg for status of traffic lights
        """
        if (self._command_status != SaverCommands.RUN):
            return

        msg = self._add_time_header_to_msg(msg)
        self._writer.write(
            "/perception/traffic_light_recognition/traffic_signals",
            serialize_message(msg),
            1
        )









