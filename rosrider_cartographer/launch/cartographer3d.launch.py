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
    rviz_config_file = os.path.join(pkg_rosrider_cartographer, 'rviz', 'rosrider_cartographer3d.rviz')
    configuration_basename_default = 'rosrider_3d.lua'

    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    occupancy_grid_resolution = LaunchConfiguration('occupancy_grid_resolution')
    octomap_resolution = LaunchConfiguration('octomap_resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    map_frame = LaunchConfiguration('map_frame')
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    octomap_max_range = LaunchConfiguration('octomap_max_range')
    octomap_min_z = LaunchConfiguration('octomap_min_z')
    octomap_max_z = LaunchConfiguration('octomap_max_z')
    launch_rviz = LaunchConfiguration('launch_rviz')

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
            ('/imu', '/imu/data'),
            ('/odom', '/odometry/filtered_local')
        ]
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/octomap_server.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': octomap_resolution,
            'frame_id': map_frame,
            'cloud_in_topic': cloud_in_topic,
            'max_range': octomap_max_range,
            'min_z': octomap_min_z,
            'max_z': octomap_max_z,
        }.items()
    )

    occupancy_grid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': occupancy_grid_resolution,
            'publish_period_sec': publish_period_sec,
            'min_z': octomap_min_z,
            'max_z': octomap_max_z
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_cartographer3d',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir_default),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename_default),
        DeclareLaunchArgument('occupancy_grid_resolution', default_value='0.05'),
        DeclareLaunchArgument('octomap_resolution', default_value='0.1'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('cloud_in_topic', default_value='/scan/points'),
        DeclareLaunchArgument('octomap_max_range', default_value='10.0'),
        DeclareLaunchArgument('octomap_min_z', default_value='-1.0'),
        DeclareLaunchArgument('octomap_max_z', default_value='2.0'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        cartographer_node,
        occupancy_grid_launch,
        octomap_launch,
        rviz_node
    ])