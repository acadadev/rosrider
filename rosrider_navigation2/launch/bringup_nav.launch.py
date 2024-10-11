import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

ROBOT_MODEL = os.environ['ROBOT_MODEL']

'''
   launches localization, navigation, with given map, and parameters for robot, for actual robot
'''

# TODO: test launch each and see who reads /scan

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default=os.path.join(get_package_share_directory('rosrider_navigation2'), 'map', 'incubator.yaml'))
    nav2_param_file = LaunchConfiguration('params_file', default=os.path.join(get_package_share_directory('rosrider_navigation2'), 'param', 'nav2_params_' + ROBOT_MODEL + '.yaml'))
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    print(nav2_launch_dir)

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_file, description='full path to map file'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_file, description='full path to param file'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='use simulation time if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={'map': map_file, 'use_sim_time': use_sim_time, 'params_file': nav2_param_file}.items(),
        )
    ])
