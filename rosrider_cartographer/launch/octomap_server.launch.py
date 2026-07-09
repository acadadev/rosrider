from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    octomap_resolution = LaunchConfiguration('octomap_resolution', default='0.1')
    frame_id = LaunchConfiguration('frame_id', default='map')
    cloud_in_topic = LaunchConfiguration('cloud_in_topic', default='/scan/points')
    max_range = LaunchConfiguration('max_range', default='10.0')
    min_z = LaunchConfiguration('min_z', default='0.0')
    max_z = LaunchConfiguration('max_z', default='2.0')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('octomap_resolution', default_value=octomap_resolution),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('cloud_in_topic', default_value=cloud_in_topic),
        DeclareLaunchArgument('max_range', default_value=max_range),
        DeclareLaunchArgument('min_z', default_value=min_z),
        DeclareLaunchArgument('max_z', default_value=max_z),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': octomap_resolution,
                'frame_id': frame_id,
                'base_frame_id': frame_id,
                'sensor_model.max_range': max_range,
                'filter_ground': False,
                'occupancy_min_z': min_z,
                'occupancy_max_z': max_z,
                'publish_free_space': True,
                'latch': False,
            }],
            remappings=[
                ('cloud_in', cloud_in_topic),
            ]),
    ])