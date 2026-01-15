import os 
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory('real_robot')
    launch_dir = os.path.join(package_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    load_state_filename = LaunchConfiguration('load_state_filename')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    carto_config = LaunchConfiguration('carto_config')
    xacro_config_file = LaunchConfiguration('xacro_config_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation if true !'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir,'config','nav2_params.yaml'),
        description='Full path to ROS2 param file to use for all launch nodes'
    )

    declare_load_state_filename_cmd = DeclareLaunchArgument(
        'load_state_filename',
        default_value=os.path.join(package_dir, '3.pbstream'),
        description='Full path to .pbstream file for Cartographer'
    )

    declare_carto_config_cmd = DeclareLaunchArgument(
        'carto_config',
        default_value='my_lidar_2d_load.lua',
        description='Full path to config file for Cartographer'
    )

    declare_xacro_config_file_cmd = DeclareLaunchArgument(
        'xacro_config_file',
        default_value=os.path.join(package_dir,'description','robot.urdf.xacro'),
        description='Full path to xacro file'
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'xacro_file' : xacro_config_file
        }.items()
    )

    carto_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_carto.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'load_state_filename': load_state_filename,
            'configuration_basename': carto_config
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'twist_mux.launch.py'))
    )

    ld = LaunchDescription()
    

    ld.add_action(declare_carto_config_cmd)
    ld.add_action(declare_load_state_filename_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_xacro_config_file_cmd)


    ld.add_action(robot_state_pub)
    ld.add_action(carto_localization)
    ld.add_action(navigation)
    ld.add_action(twist_mux)

    return ld