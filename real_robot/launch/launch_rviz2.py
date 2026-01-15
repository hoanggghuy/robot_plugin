import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'real_robot'
    default_rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'my_robot.rviz'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rviz_config', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node
    ])