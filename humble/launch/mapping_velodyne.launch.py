import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('fast_lio')
    parameter_file = os.path.join(share_dir, 'config', 'velodyne.yaml')
    rviz_config_file = os.path.join(share_dir, 'rviz_cfg', 'loam_livox.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[parameter_file, {
            'feature_extract_enable': False,
            'point_filter_num': 4,
            'max_iteration': 3,
            'filter_size_surf': 0.5,
            'filter_size_map': 0.5,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        fast_lio_node,
        rviz_node
    ])
