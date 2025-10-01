import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    pkg_rosrider_cartographer = get_package_share_directory('rosrider_cartographer')
    cartographer_config_dir_default = os.path.join(pkg_rosrider_cartographer, 'config')
    rviz_config_file = os.path.join(pkg_rosrider_cartographer, 'rviz', 'rosrider_cartographer.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    launch_rviz = LaunchConfiguration('launch_rviz')

    if use_sim_time:
        configuration_basename_default = 'rosrider_3d_gazebo.lua'
    else:
        configuration_basename_default = 'rosrider_3d.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/points2', '/scan/points'),
            ('/imu', '/imu/data')
        ]
    )

    occupancy_grid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir_default, description='Full path to the Cartographer config directory'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename_default, description='Name of the Lua configuration file for Cartographer'),
        DeclareLaunchArgument('resolution', default_value='0.05', description='Resolution of a grid cell in the occupancy grid'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0', description='Occupancy grid publishing period in seconds'),
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Whether to launch RViz2'),
        cartographer_node,
        occupancy_grid_launch,
        rviz_node
    ])