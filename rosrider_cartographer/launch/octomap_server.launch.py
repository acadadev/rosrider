from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    octomap_resolution = LaunchConfiguration('octomap_resolution')
    frame_id = LaunchConfiguration('frame_id')
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    max_range = LaunchConfiguration('max_range')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')

    return LaunchDescription([

        DeclareLaunchArgument('octomap_resolution', default_value='0.1'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('cloud_in_topic', default_value='/scan/points'),
        DeclareLaunchArgument('max_range', default_value='10.0'),
        DeclareLaunchArgument('min_z', default_value='-1.0'),
        DeclareLaunchArgument('max_z', default_value='2.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

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