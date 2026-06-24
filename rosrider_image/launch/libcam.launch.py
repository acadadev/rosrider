import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(get_package_share_directory('rosrider_image'), 'param', 'libcam.yaml')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='LD_PRELOAD',
            value='/usr/local/lib/aarch64-linux-gnu/libcamera.so.0.7:/usr/local/lib/aarch64-linux-gnu/libcamera-base.so.0.7:/usr/local/lib/aarch64-linux-gnu/libpisp.so.1'
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            namespace='camera',
            name='camera_node',
            parameters=[
                config,
                {
                    'width': 640,
                    'height': 400,
                    'format': 'YUYV'
                }
            ],
            remappings=[
                ('~/image_raw', '/camera/image_raw'),
                ('~/image_raw/compressed', '/camera/image_raw/compressed'),
                ('~/camera_info', '/camera/camera_info')
            ]
        )
    ])