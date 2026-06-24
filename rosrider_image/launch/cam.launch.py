import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

config = os.path.join(get_package_share_directory('rosrider_image'), 'param', 'cam.yaml')

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['v4l2-ctl', '-d', '/dev/video0', '-p', '10'],
            shell=False
        ),
        Node(
            package='v4l2_camera',
            namespace='camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[config]
        )
    ])


