# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')
    autoware_launch_pkg_prefix = get_package_share_directory('autoware_auto_launch')

    euclidean_cluster_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/ray_ground_classifier.param.yaml')
    scan_downsampler_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/scan_downsampler.param.yaml')

    lanelet2_map_provider_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/lanelet2_map_provider.param.yaml')

    lane_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/lane_planner.param.yaml')
    costmap_generator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/costmap_generator.param.yaml')
    freespace_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/freespace_planner.param.yaml')
    object_collision_estimator_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/object_collision_estimator.param.yaml')
    behavior_planner_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/behavior_planner.param.yaml')
    off_map_obstacles_filter_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/off_map_obstacles_filter.param.yaml')

    multi_object_tracker_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/multi_object_tracker.param.yaml')
    covariance_insertion_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ndt_smoothing/ndt_covariance_override.param.yaml')
    state_estimation_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ndt_smoothing/tracking.param.yaml')
    prediction_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/prediction.param.yaml')
    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/vehicle_characteristics.param.yaml')
    vehicle_constants_manager_param_file = os.path.join(
        autoware_launch_pkg_prefix, 'param/lexus_rx_hybrid_2016.param.yaml')

    point_cloud_fusion_node_pkg_prefix = get_package_share_directory(
        'point_cloud_fusion_nodes')

    map_osm_file = os.path.join(
        avp_demo_pkg_prefix, 'data/autonomoustuff_parking_lot.osm')

    # Arguments

    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='True',
        description='Enable obstacle detection'
    )
    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=lane_planner_param_file,
        description='Path to parameter file for lane planner'
    )
    costmap_generator_param = DeclareLaunchArgument(
        'costmap_generator_param_file',
        default_value=costmap_generator_param_file,
        description='Path to parameter file for costmap generator'
    )
    freespace_planner_param = DeclareLaunchArgument(
        'freespace_planner_param_file',
        default_value=freespace_planner_param_file,
        description='Path to parameter file for freespace_planner'
    )
    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=object_collision_estimator_param_file,
        description='Path to parameter file for object collision estimator'
    )
    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to parameter file for behavior planner'
    )
    off_map_obstacles_filter_param = DeclareLaunchArgument(
        'off_map_obstacles_filter_param_file',
        default_value=off_map_obstacles_filter_param_file,
        description='Path to parameter file for off-map obstacle filter'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )
    vehicle_constants_manager_param = DeclareLaunchArgument(
        'vehicle_constants_manager_param_file',
        default_value=vehicle_constants_manager_param_file,
        description='Path to parameter file for vehicle_constants_manager'
    )
    multi_object_tracker_param = DeclareLaunchArgument(
        'multi_object_tracker_param_file',
        default_value=multi_object_tracker_param_file,
        description='Path to config file for multiple object tracker'
    )
    state_estimation_param = DeclareLaunchArgument(
        'state_estimation_param_file',
        default_value=state_estimation_param_file,
        description='Path to config file for state estimator'
    )
    covariance_insertion_param = DeclareLaunchArgument(
        'covariance_insertion_param_file',
        default_value=covariance_insertion_param_file,
        description='Path to config file for covariance insertion'
    )
    prediction_param = DeclareLaunchArgument(
        'prediction_param_file',
        default_value=prediction_param_file,
        description='Path to config file for prediction'
    )

    # Nodes

    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        executable='euclidean_cluster_node_exe',
        namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[
            ("points_in", "points_nonground")
        ]
    )
    # point cloud fusion runner to fuse front and rear lidar

    point_cloud_fusion_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(point_cloud_fusion_node_pkg_prefix,
                             'launch/vlp16_sim_lexus_pc_fusion.launch.py'))
    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[("points_in", "/lidars/points_fused")]
    )
    scan_downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidars',
        name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ]
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file'),
                    {'map_osm_file': map_osm_file}]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )
    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )
    lane_planner = Node(
        package='lane_planner_nodes',
        name='lane_planner_node',
        namespace='planning',
        executable='lane_planner_node_exe',
        parameters=[
            LaunchConfiguration('lane_planner_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )
    costmap_generator = Node(
        package='costmap_generator_nodes',
        executable='costmap_generator_node_exe',
        name='costmap_generator_node',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('costmap_generator_param_file'),
        ],
        remappings=[
            ('~/client/HAD_Map_Service', '/had_maps/HAD_Map_Service')
        ]
    )
    freespace_planner = Node(
        package='freespace_planner_nodes',
        executable='freespace_planner_node_exe',
        name='freespace_planner',
        namespace='planning',
        output='screen',
        parameters=[
            LaunchConfiguration('freespace_planner_param_file'),
            LaunchConfiguration('vehicle_constants_manager_param_file')
        ]
    )
    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        name='object_collision_estimator_node',
        namespace='planning',
        executable='object_collision_estimator_node_exe',
        parameters=[
            LaunchConfiguration('object_collision_estimator_param_file'),
            {
                'target_frame_id': "map"
            },
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
            ('predicted_objects', '/prediction/predicted_objects'),

        ],
        condition=IfCondition(LaunchConfiguration('with_obstacles'))
    )
    behavior_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[
            LaunchConfiguration('behavior_planner_param_file'),
            {'enable_object_collision_estimator': LaunchConfiguration('with_obstacles')},
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('gear_report', '/vehicle/gear_report'),
            ('gear_command', '/vehicle/gear_command')
        ]
    )
    off_map_obstacles_filter = Node(
        package='off_map_obstacles_filter_nodes',
        name='off_map_obstacles_filter_node',
        namespace='perception',
        executable='off_map_obstacles_filter_nodes_exe',
        parameters=[LaunchConfiguration('off_map_obstacles_filter_param_file')],
        output='screen',
        remappings=[
            ('bounding_boxes_in', 'lidar_bounding_boxes'),
            ('bounding_boxes_out', 'lidar_bounding_boxes_filtered'),
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
        ]
    )

    multi_object_tracker = Node(
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker',
        namespace='perception',
        package='tracking_nodes',
        output='screen',
        parameters=[
            LaunchConfiguration('multi_object_tracker_param_file'),
            {
                'use_ndt': True,
                'track_frame_id': "map",
                'use_vision': False,
                'visualize_track_creation': False
            },
        ],
        remappings=[
            ("detected_objects", "/lidars/lidar_detected_objects"),
            ("ego_state", "/localization/odometry"),
            ("clusters", "/perception/points_clustered")
        ],
        condition=IfCondition(LaunchConfiguration('with_obstacles'))
    )
    state_estimation = Node(
        executable='state_estimation_node_exe',
        name='state_estimation',
        namespace='localization',
        output="screen",
        package='state_estimation_nodes',
        parameters=[
            LaunchConfiguration('state_estimation_param_file'),
        ],
        remappings=[
            ("filtered_state", "/localization/odometry"),
        ],
        condition=IfCondition(LaunchConfiguration('with_obstacles'))
    )
    covariance_insertion = Node(
        executable='covariance_insertion_node_exe',
        name='covariance_insertion',
        namespace='localization',
        output="screen",
        package='covariance_insertion_nodes',
        parameters=[
            LaunchConfiguration('covariance_insertion_param_file'),
        ],
        remappings=[
            ("messages", "/localization/ndt_pose"),
            ("messages_with_overriden_covariance", "ndt_pose_with_covariance")
        ],
        condition=IfCondition(LaunchConfiguration('with_obstacles'))
    )
    prediction = Node(
        executable='prediction_nodes_node_exe',
        name='prediction',
        namespace='prediction',
        output="screen",
        package='prediction_nodes',
        parameters=[LaunchConfiguration('prediction_param_file')],
        remappings=[
            ("tracked_objects", "/perception/tracked_objects")
        ],
        condition=IfCondition(LaunchConfiguration('with_obstacles'))
    )

    return LaunchDescription([
        euclidean_cluster_param,
        ray_ground_classifier_param,
        scan_downsampler_param,
        with_obstacles_param,
        lanelet2_map_provider_param,
        lane_planner_param,
        costmap_generator_param,
        freespace_planner_param,
        object_collision_estimator_param,
        behavior_planner_param,
        off_map_obstacles_filter_param,
        vehicle_characteristics_param,
        vehicle_constants_manager_param,
        euclidean_clustering,
        ray_ground_classifier,
        scan_downsampler,
        point_cloud_fusion_node,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        global_planner,
        lane_planner,
        costmap_generator,
        freespace_planner,
        object_collision_estimator,
        behavior_planner,
        off_map_obstacles_filter,
        multi_object_tracker_param,
        multi_object_tracker,
        state_estimation_param,
        state_estimation,
        covariance_insertion_param,
        covariance_insertion,
        prediction_param,
        prediction
    ])
