import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

'''
    ros2 launch slam_toolbox online_sync_launch.py

    autostart = true
    use_lifecycle_manager = false
    use_sim_time = true
    slam_params_file = mapper_params_next.yaml

'''

ROBOT_MODEL = os.environ['ROBOT_MODEL']


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # notice: we use same parameter file for both real world and simulation

    params_file = LaunchConfiguration('params_file',  default=os.path.join(get_package_share_directory('rosrider_nav'), 'param', 'mapper_params_' + ROBOT_MODEL + '.yaml'))

    slam2_launch_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file, description='full path to slam parameters file'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='use simulation time if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam2_launch_dir, '/online_sync_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': params_file}.items(),
        )
    ])
