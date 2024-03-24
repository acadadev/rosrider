from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

'''
TODO: ability to have own teleop launch with own config instead of the systems.
'''

def generate_launch_description():

    ROBOT_MODEL = os.environ['ROBOT_MODEL']

    urdf = os.path.join(get_package_share_directory('rosrider_description'), 'urdf', ROBOT_MODEL + '.urdf')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='rosrider_node',
            executable='rosrider_node',
            name='rosrider_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('rosrider_node'), 'param', ROBOT_MODEL + '.yaml')],
           )
])
