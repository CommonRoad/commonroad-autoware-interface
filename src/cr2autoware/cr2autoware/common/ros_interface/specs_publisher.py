"""
This file holds specification descriptions for all topic publishers of CR2Auto node.
To add a new topic publisher, add the specification here first.
"""

# ROS imports
from rclpy.qos import QoSDurabilityPolicy

# ROS messages
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

# Autoware messages
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.msg import RouteState  # type: ignore

# Tier IV message imports
from tier4_planning_msgs.msg import VelocityLimit   # type: ignore

# cr2autoware imports
from .specs_base import PublisherSpec
from .specs_base import QosSpec


# =========================================


# publish goal pose
goal_pose_pub = PublisherSpec(name="/planning/mission_planning/goal",
                              msg_type=PoseStamped,
                              depth=1)


# publish trajectory
traj_pub = PublisherSpec(name="/planning/commonroad/trajectory",
                         msg_type=AWTrajectory,
                         depth=1)


# publish autoware state
aw_state_pub = PublisherSpec(name="/autoware/state",
                             msg_type=AutowareState,
                             depth=1)


# publish vehicle engage for AW Planning Simulation
vehicle_engage_pub = PublisherSpec(name="/vehicle/engage",
                                   msg_type=Engage,
                                   depth=1)


# publish engage required by node /control/operation_mode_transistion_manager
api_engage_pub = PublisherSpec(name="/api/autoware/get/engage",
                               msg_type=Engage,
                               depth=1)


# publish routing state
routing_state_pub = PublisherSpec(name="/api/routing/state",
                                  msg_type=RouteState,
                                  depth=1,
                                  qos_profile=QosSpec(QoSDurabilityPolicy.TRANSIENT_LOCAL))


# publish route marker
route_pub = PublisherSpec(name="/planning/mission_planning/route_marker",
                          msg_type=MarkerArray,
                          depth=1,
                          qos_profile=QosSpec(QoSDurabilityPolicy.TRANSIENT_LOCAL))


# publish reference trajectory to motion velocity smoother
velocity_pub = PublisherSpec(name="/planning/scenario_planning/scenario_selector/trajectory",
                             msg_type=AWTrajectory,
                             depth=1)


# publish initial state of the scenario (replay solution trajectory mode)
initial_pose_pub = PublisherSpec(name="/initialpose3d",
                                 msg_type=PoseWithCovarianceStamped,
                                 depth=1)


# publish goal region(s) of the scenario
goal_region_pub = PublisherSpec(name="/goal_region_marker_array",
                                msg_type=MarkerArray,
                                depth=1)


# publish max velocity limit
velocity_limit_pub = PublisherSpec(name="/planning/scenario_planning/max_velocity",
                                   msg_type=VelocityLimit,
                                   depth=1)


# publish current velocity limit for display in RVIZ
velocity_limit_pub_vis = PublisherSpec(name="/planning/scenario_planning/current_max_velocity",
                                       msg_type=VelocityLimit,
                                       depth=1,
                                       qos_profile=QosSpec(QoSDurabilityPolicy.TRANSIENT_LOCAL))