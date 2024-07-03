"""
ROS Interface Subscription Specification
========================================

This file holds specification descriptions for all topic subscriptions of CR2Auto node.

To add a new topic subscription, add the specification here first.

---------------------------
**Subscriptions**:

* spec_initial_pose_sub:
    * Description: subscribe initial pose
    * Topic: `/initialpose3d`
    * Message Type: `geometry_msgs.msg.PoseWithCovarianceStamped`
* spec_echo_back_goal_pose_sub:
    * Description: subscribe echo back goal pose
    * Topic: `/planning/mission_planning/echo_back_goal_pose`
    * Message Type: `geometry_msgs.msg.PoseStamped`
* spec_goal_pose_sub:
    * Description: subscribe goal pose
    * Topic: `/planning/mission_planning/goal`
    * Message Type: `geometry_msgs.msg.PoseStamped`
* spec_auto_button_sub:
    * Description: subscribe autoware engage message
    * Topic: `/autoware/engage`
    * Message Type: `autoware_auto_vehicle_msgs.msg.Engage`
* spec_velocity_limit_sub:
    * Description: subscribe velocity limit from API (Note: Here we directly use the value from the API velocity limit
    setter in RVIZ.)
    * Topic: `/planning/scenario_planning/max_velocity_default`
    * Message Type: `tier4_planning_msgs.msg.VelocityLimit`
* spec_routing_state_sub:
    * Description: subscribe routing state
    * Topic: `/api/routing/state`
    * Message Type: `autoware_adapi_v1_msgs.msg.RouteState`
* spec_autoware_state_sub:
    * Description: subscribe autoware state
    * Topic: `/autoware/state`
    * Message Type: `autoware_auto_system_msgs.msg.AutowareState`
* spec_odometry:
    * Description: subscribe current state from odometry (kinematic state)
    * Topic: `/localization/kinematic_state`
    * Message Type: `nav_msgs.msg.Odometry`
* spec_curr_acc:
    * Description: subscribe current acceleration (separate topic, currently not in /localization/kinematic_state)
    * Topic: `/localization/acceleration`
    * Message Type: `geometry_msgs.msg.AccelWithCovarianceStamped`
* spec_objects_sub:
    * Description: subscribe predicted objects from perception
    * Topic: `/perception/object_recognition/objects`
    * Message Type: `autoware_auto_perception_msgs.msg.PredictedObjects`
"""

# ROS messages
from geometry_msgs.msg import PoseStamped # type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped # type: ignore
from geometry_msgs.msg import AccelWithCovarianceStamped # type: ignore
from nav_msgs.msg import Odometry # type: ignore

# Autoware messages
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects  # type: ignore
from autoware_auto_perception_msgs.msg import TrafficSignalArray  # type: ignore
from autoware_auto_planning_msgs.msg import Trajectory # type: ignore

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.msg import RouteState  # type: ignore

# Tier IV message imports
from tier4_planning_msgs.msg import VelocityLimit  # type: ignore

# cr2autoware imports
from .specs_base import SubscriptionSpec

# =========================================

# subscribe initial pose
spec_initial_pose_sub = SubscriptionSpec(name="/initialpose3d",
                                         msg_type=PoseWithCovarianceStamped,
                                         depth=1)

# subscribe echo back goal pose
spec_echo_back_goal_pose_sub = SubscriptionSpec(name="/planning/mission_planning/echo_back_goal_pose",
                                                msg_type=PoseStamped,
                                                depth=1)

# subscribe goal pose
spec_goal_pose_sub = SubscriptionSpec(name="/planning/mission_planning/goal",
                                      msg_type=PoseStamped,
                                      depth=1)



# subscribe autoware engage message
spec_auto_button_sub = SubscriptionSpec(name="/autoware/engage",
                                        msg_type=Engage,
                                        depth=1)

# subscribe velocity limit from API (Note: Here we directly use the value from the API velocity limit setter in RVIZ.)
spec_velocity_limit_sub = SubscriptionSpec(name="/planning/scenario_planning/max_velocity_default",
                                           msg_type=VelocityLimit,
                                           depth=1)

# subscribe routing state
spec_routing_state_sub = SubscriptionSpec(name="/api/routing/state",
                                          msg_type=RouteState,
                                          depth=1)

# subscribe autoware state
spec_autoware_state_sub = SubscriptionSpec(name="/autoware/state",
                                           msg_type=AutowareState,
                                           depth=1)

# subscribe current state from odometry (kinematic state)
spec_odometry = SubscriptionSpec(name="/localization/kinematic_state",
                                 msg_type=Odometry,
                                 depth=1)

# subscribe current acceleration (separate topic, currently not in /localization/kinematic_state)
spec_curr_acc = SubscriptionSpec(name="/localization/acceleration",
                                 msg_type=AccelWithCovarianceStamped,
                                 depth=1)

# subscribes to smoothed trajectory
spec_traj_smoothed = SubscriptionSpec(
    name="/planning/scenario_planning/trajectory_smoothed",
    msg_type=Trajectory,
    depth=1
)

# subscribes to trajectory
spec_traj = SubscriptionSpec(
    name="/planning/scenario_planning/trajectory",
    msg_type=Trajectory,
    depth=1
)

spec_objects_sub = SubscriptionSpec(
    name="/perception/object_recognition/objects",
    msg_type=PredictedObjects,
    depth=1
)


# TODO: merge with above and check in rosbag_saving_handler etc.
# subscribe traffic signals from perception
spec_traffic_lights = SubscriptionSpec(name="/perception/traffic_light_recognition/traffic_signals",
                                            msg_type=TrafficSignalArray,
                                            depth=1)




# subscribes to traffic lights
spec_traffic_lights = SubscriptionSpec(
    name="/perception/traffic_light_recognition/traffic_signals",
    msg_type=TrafficSignalArray,
    depth=1
)

# TODO: Check data saving and merge
spec_obj_recognition = spec_objects_sub
spec_traffic_signals_sub = spec_traffic_lights



