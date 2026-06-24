import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('image_rect', '/image_rect')
        ]
    )

    fix_p_matrix_rect_node = Node(
        package='rosrider_image',
        executable='fix_p_matrix_rect.py',
        name='fix_p_matrix_rect',
        output='screen',
        parameters=[
            {'use_sim_time': False },
        ]
    )

    return LaunchDescription([
        rectify_node
    ])