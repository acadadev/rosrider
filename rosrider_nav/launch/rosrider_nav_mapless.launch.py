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

def generate_launch_description():

    pkg_rosrider_nav = get_package_share_directory('rosrider_nav')

    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_rosrider_nav, 'param', 'nav2_params_mapless_' + ROBOT_MODEL + '.yaml'))
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapless',
        arguments=['-d', os.path.join(pkg_rosrider_nav, 'rviz', 'rosrider_mapless.rviz')],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
            launch_arguments={'use_sim_time': 'true', 'params_file': params_file}.items(),
        )
    ])

# TODO: use_sim_time ambigious

