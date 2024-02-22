"""
This file holds specification descriptions for all topic subscriptions of CR2Auto node.
To add a new topic subscription, add the specification here first.
"""

# ROS messages
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# Autoware messages
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.msg import RouteState # type: ignore

# Tier IV message imports
from tier4_planning_msgs.msg import VelocityLimit   # type: ignore

# cr2autoware imports
from .specs_base import SubscriptionSpec


# =========================================

# subscribe initial pose
initial_pose_sub = SubscriptionSpec(name="/initialpose3d",
                                    msg_type=PoseWithCovarianceStamped,
                                    depth=1)


# subscribe goal pose
goal_pose_sub = SubscriptionSpec(name="/planning/mission_planning/goal",
                                msg_type=PoseStamped,
                                depth=1)


# subscribe autoware engage message
auto_button_sub = SubscriptionSpec(name="/autoware/engage",
                                   msg_type=Engage,
                                   depth=1)


# subscribe velocity limit from API
velocity_limit_sub = SubscriptionSpec(name="/planning/scenario_planning/max_velocity_default",
                                      msg_type=VelocityLimit,
                                      depth=1)


# subscribe routing state
routing_state_sub = SubscriptionSpec(name="/api/routing/state",
                                     msg_type=RouteState,
                                     depth=1)


# subscribe autoware state
autoware_state_sub = SubscriptionSpec(name="/autoware/state",
                                      msg_type=AutowareState,
                                      depth=1)
