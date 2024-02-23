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
from dummy_perception_publisher.msg import Object  # type: ignore

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.msg import RouteState  # type: ignore

# Tier IV message imports
from tier4_planning_msgs.msg import VelocityLimit  # type: ignore

# cr2autoware imports
from .specs_base import PublisherSpec
from .specs_base import QosSpec

# =========================================


# publish goal pose
spec_goal_pose_pub = PublisherSpec(name="/planning/mission_planning/goal",
                                   msg_type=PoseStamped,
                                   depth=1)

# publish trajectory (Note: We do publish the output trajectory to the planning_validator.)
spec_traj_pub = PublisherSpec(name="/planning/commonroad/trajectory",
                              msg_type=AWTrajectory,
                              depth=1)

# publish autoware state (https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_system_msgs/msg/AutowareState.idl)
spec_aw_state_pub = PublisherSpec(name="/autoware/state",
                                  msg_type=AutowareState,
                                  depth=1)

# publish vehicle engage for AW Planning Simulation
spec_vehicle_engage_pub = PublisherSpec(name="/vehicle/engage",
                                        msg_type=Engage,
                                        depth=1)

# publish engage required by node /control/operation_mode_transistion_manager
spec_api_engage_pub = PublisherSpec(name="/api/autoware/get/engage",
                                    msg_type=Engage,
                                    depth=1)

# publish routing state
spec_routing_state_pub = PublisherSpec(name="/api/routing/state",
                                       msg_type=RouteState,
                                       depth=1,
                                       qos_profile=QosSpec(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

# publish route marker
spec_route_pub = PublisherSpec(name="/planning/mission_planning/route_marker",
                               msg_type=MarkerArray,
                               depth=1,
                               qos_profile=QosSpec(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

# publish reference trajectory to motion velocity smoother
spec_velocity_pub = PublisherSpec(name="/planning/scenario_planning/scenario_selector/trajectory",
                                  msg_type=AWTrajectory,
                                  depth=1)

# publish initial state of the scenario (replay solution trajectory mode)
spec_initial_pose_pub = PublisherSpec(name="/initialpose3d",
                                      msg_type=PoseWithCovarianceStamped,
                                      depth=1)

# publish goal region(s) of the scenario (TODO: use correct topic)
spec_goal_region_pub = PublisherSpec(name="/goal_region_marker_array",
                                     msg_type=MarkerArray,
                                     depth=1)

# publish max velocity limit
spec_velocity_limit_pub = PublisherSpec(name="/planning/scenario_planning/max_velocity",
                                        msg_type=VelocityLimit,
                                        depth=1)

# publish current velocity limit for display in RVIZ
spec_velocity_limit_pub_vis = PublisherSpec(name="/planning/scenario_planning/current_max_velocity",
                                            msg_type=VelocityLimit,
                                            depth=1,
                                            qos_profile=QosSpec(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

# publish initial pose (when loading scenario from a CR file)
spec_initial_pose_2d_pub = PublisherSpec(name="/initialpose",
                                         msg_type=PoseWithCovarianceStamped,
                                         depth=1)

# obstacle publisher for dynamic obstacle in CR scenario (when loading scenario from a CR file)
spec_obstacle_pub = PublisherSpec(name="/simulation/dummy_perception_publisher/object_info",
                                  msg_type=Object,
                                  depth=1)
