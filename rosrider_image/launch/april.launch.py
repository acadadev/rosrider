import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    pkg_rosrider_image = get_package_share_directory('rosrider_image')

    os.environ["IMAGE_TRANSPORT_TO_EXCLUDE"] = "compressedDepth,theora,zstd"

    fix_p_matrix_rect_node = Node(
        package='rosrider_image',
        executable='fix_p_matrix_rect.py',
        name='fix_p_matrix_rect',
        output='screen',
        parameters=[
            {'use_sim_time': False },
        ]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image', '/image_rect'),
            ('camera_info', '/camera_info')
        ],
        parameters=[
            os.path.join(pkg_rosrider_image, 'param', 'apriltag.yaml'),
            {'use_sim_time': False }
        ],
        output='screen'
    )

    apriltag_overlay = Node(
        package='rosrider_image',
        executable='apriltag_overlay.py',
        name='apriltag_overlay',
        parameters=[
            {'use_sim_time': False },
        ],
        output='screen'
    )

    apriltag_localization = Node(
        package='rosrider_gz_bringup',
        executable='apriltag_localization.py',
        name='apriltag_localization',
        parameters=[
            {'use_sim_time': False },
        ],
        output='screen'
    )

    apriltag_navigation = Node(
        package='rosrider_gz_bringup',
        executable='apriltag_navigation.py',
        name='apriltag_navigation',
        parameters=[
            {'use_sim_time': False },
        ],
        output='screen'
    )

    return LaunchDescription([
        #fix_p_matrix_rect_node,
        apriltag_node,
        apriltag_overlay,
        apriltag_localization,
        #apriltag_navigation
    ])