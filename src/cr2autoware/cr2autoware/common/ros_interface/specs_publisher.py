"""
ROS Interface Publisher Specification
=====================================

This file holds specification descriptions for all topic publishers of CR2Auto node.

To add a new topic publisher, add the specification here first.

---------------------------
**Publishers**:

* spec_goal_pose_pub:
    * Description: publish goal pose
    * Topic: `/planning/mission_planning/goal`
    * Message Type: `geometry_msgs.msg.PoseStamped`
* spec_traj_pub:
    * Description: publish trajectory (Note: We do publish the output trajectory to the planning_validator.)
    * Topic: `/planning/commonroad/trajectory`
    * Message Type: `autoware_auto_planning_msgs.msg.Trajectory`
* spec_aw_state_pub:
    * Description: publish autoware state, see [AutowareStateDocumentation](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_system_msgs/msg/AutowareState.idl)
    * Topic: `/autoware/state`
    * Message Type: `autoware_auto_system_msgs.msg.AutowareState`
* spec_vehicle_engage_pub:
    * Description: publish vehicle engage for AW Planning Simulation
    * Topic: `/vehicle/engage`
    * Message Type: `autoware_auto_vehicle_msgs.msg.Engage`
* spec_api_engage_pub:
    * Description: publish engage required by node /control/operation_mode_transistion_manager
    * Topic: `/api/autoware/get/engage`
    * Message Type: `autoware_auto_vehicle_msgs.msg.Engage`
* spec_routing_state_pub:
    * Description: publish routing state
    * Topic: `/api/routing/state`
    * Message Type: `autoware_adapi_v1_msgs.msg.RouteState`
* spec_route_pub:
    * Description: publish route marker
    * Topic: `/planning/mission_planning/route_marker`
    * Message Type: `visualization_msgs.msg.MarkerArray`
* spec_velocity_pub:
    * Description: publish reference trajectory to motion velocity smoother
    * Topic: `/planning/scenario_planning/scenario_selector/trajectory`
    * Message Type: `autoware_auto_planning_msgs.msg.Trajectory`
* spec_initial_pose_pub:
    * Description: publish initial state of the scenario (replay solution trajectory mode)
    * Topic: `/initialpose3d`
    * Message Type: `geometry_msgs.msg.PoseWithCovarianceStamped`
* spec_goal_region_pub:
    * Description: publish goal region(s) of the scenario
    * Topic: `/goal_region_marker_array`
    * Message Type: `visualization_msgs.msg.MarkerArray`
* spec_velocity_limit_pub:
    * Description: publish max velocity limit
    * Topic: `/planning/scenario_planning/max_velocity`
    * Message Type: `tier4_planning_msgs.msg.VelocityLimit`
* spec_velocity_limit_pub_vis:
    * Description: publish current velocity limit for display in RVIZ
    * Topic: `/planning/scenario_planning/current_max_velocity`
    * Message Type: `tier4_planning_msgs.msg.VelocityLimit`
* spec_initial_pose_2d_pub:
    * Description: publish initial pose (when loading scenario from a CR file)
    * Topic: `/initialpose`
    * Message Type: `geometry_msgs.msg.PoseWithCovarianceStamped`
* spec_obstacle_pub:
    * Description: obstacle publisher for dynamic obstacle in CR scenario (when loading scenario from a CR file)
    * Topic: `/simulation/dummy_perception_publisher/object_info`
    * Message Type: `dummy_perception_publisher.msg.Object`
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

