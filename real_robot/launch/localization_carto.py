import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'real_robot' # Đổi tên package của bạn nếu khác
    pkg_dir = get_package_share_directory(pkg_name)
    
    # QUAN TRỌNG: Trỏ vào file lua localization bạn vừa cấu hình (có pure_localization_trimmer)
    configuration_basename = 'my_lidar_2d_load.lua' 

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation if true'
    )
    
    # Tham số đường dẫn file map .pbstream
    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename',
        # Bạn có thể set đường dẫn mặc định cứng ở đây nếu muốn đỡ phải gõ mỗi lần chạy
        default_value=os.path.join(os.environ['HOME'], 'my_map.pbstream'), 
        description='Full path to the .pbstream file to load.'
    )

    load_state_filename = LaunchConfiguration('load_state_filename')
    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'config'),
            '-configuration_basename', configuration_basename,
            '-load_state_filename', load_state_filename # Bắt buộc phải có dòng này để load map
        ],
        remappings=[
            ('echoes', 'horizontal_laser_2d'),
            ('scan', 'scan') ,
            ('initialpose', '/initialpose'),
        ]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.03
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        load_state_filename_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])