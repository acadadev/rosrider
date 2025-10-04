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
    ros2 launch nav2_bringup navigation_launch.py
   
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
   
'''

ROBOT_MODEL = os.environ['ROBOT_MODEL']

def generate_launch_description():

    # TODO: do not forget keeoout
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(get_package_share_directory('rosrider_nav'), 'param', 'nav2_keepout_params_' + ROBOT_MODEL + '.yaml'))

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file, description='full path to navigation params file'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='use simulation time if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file}.items(),
        )
    ])

