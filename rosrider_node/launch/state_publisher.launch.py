import os
import yaml
import pathlib

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ROBOT_MODEL = os.environ['ROBOT_MODEL']

    urdf_file = os.path.join(get_package_share_directory('rosrider_description'), 'urdf', ROBOT_MODEL + '.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        )
])
