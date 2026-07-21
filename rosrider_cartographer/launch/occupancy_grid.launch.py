from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('min_z', default_value='-1.0'),
        DeclareLaunchArgument('max_z', default_value='2.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec,
                '-min_z', min_z,
                '-max_z', max_z,
            ]),
    ])