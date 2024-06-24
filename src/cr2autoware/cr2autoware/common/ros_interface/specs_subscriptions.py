"""
This file holds specification descriptions for all topic subscriptions of CR2Auto node.
To add a new topic subscription, add the specification here first.
"""

# ROS messages
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry

# Autoware messages
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore
from autoware_auto_planning_msgs.msg import Trajectory # type: ignore
from autoware_auto_perception_msgs.msg import PredictedObjects # type: ignore
from autoware_auto_perception_msgs.msg import TrafficSignalArray  # type: ignore

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


# subscribe predicted objects from perception
spec_objects_sub = SubscriptionSpec(
    name="/perception/object_recognition/objects",
    msg_type=PredictedObjects,
    depth=1
)

# subscribes to traffic lights
spec_traffic_lights = SubscriptionSpec(
    name="/perception/traffic_light_recognition/traffic_signals",
    msg_type=TrafficSignalArray,
    depth=1
)
