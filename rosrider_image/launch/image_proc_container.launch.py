from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Create a managed container for the full image_proc suite
        ComposableNodeContainer(
            name='image_proc_container',
            namespace='camera',  # Puts everything neatly inside your camera's namespace
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    # Remap to your physical camera driver topics
                    remappings=[
                        ('image', '/camera/image_raw'),
                        ('camera_info', '/camera/camera_info'),
                        ('image_rect', '/image_rect'),
                    ],
                    parameters=[{'use_sim_time': True}]
                )
            ],
            output='screen'
        )
    ])