from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    cr2autoware_pkg_prefix = get_package_share_directory('cr2autoware')
    #cr2autoware_pkg_prefix = "/home/drivingsim/adehome/workspace/dfg-car/install/cr2autoware/share/cr2autoware"
    cr2autoware_param_file = os.path.join(cr2autoware_pkg_prefix, 'param/cr2autoware_param_file.param.yaml')

    # Argument

    cr2autoware_param = DeclareLaunchArgument(
        'cr2autoware_param_file',
        default_value=cr2autoware_param_file,
        description='Path to config file'
    )

    # Node
    
    cr2autoware_node = Node(
        package='cr2autoware',
        executable='cr2autoware_node',
        name='cr2autoware_node',
        parameters=[LaunchConfiguration('cr2autoware_param_file'),]
    )

    return LaunchDescription([
            cr2autoware_param,
            cr2autoware_node,
        ])
