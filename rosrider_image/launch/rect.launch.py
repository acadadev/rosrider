import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory('rosrider_image'), 'param', 'cam.yaml')

# ros2 run image_proc rectify_node --ros-args -r image:=/camera/image_raw -r camera_info:=/camera/camera_info

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_proc',
            namespace='camera',
            executable='rectify_node',
            name='rectify_node',
            parameters=[config]
        )
    ])


