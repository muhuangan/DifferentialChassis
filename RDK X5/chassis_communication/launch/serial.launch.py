import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('chassis_communication'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(
            package='chassis_communication', executable='serial_node',
            name='serial_node', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            }],
        ),
    ])
