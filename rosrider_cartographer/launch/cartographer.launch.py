import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rosrider_cartographer_prefix = get_package_share_directory('rosrider_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(rosrider_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rosrider_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('rosrider_cartographer'), 'rviz', 'rosrider_cartographer.rviz')

    return LaunchDescription([

        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir, description='full path to config file to load'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename, description='name of lua file for cartographer'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='use simulation time if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # remappings=[('/scan', '/ldlidar_node/scan')],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),
        DeclareLaunchArgument('resolution', default_value=resolution, description='resolution of grid cell in occupancy grid'),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec, description='occupancy grid publishing period'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution, 'publish_period_sec': publish_period_sec}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('launch_rviz', default='false')),
            output='screen'),
    ])
