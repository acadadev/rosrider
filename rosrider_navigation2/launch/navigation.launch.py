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
 Just launches default navigation stack, without map, and parameters for robot
'''


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_param_file = LaunchConfiguration('params_file', default=os.path.join(get_package_share_directory('rosrider_navigation2'), 'param', 'nav2_params_' + ROBOT_MODEL + '.yaml'))
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=nav2_param_file, description='full path to param file'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='use simulation time if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_param_file}.items(),
        )
    ])