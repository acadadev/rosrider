from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    min_z = LaunchConfiguration('min_z', default='0.0')
    max_z = LaunchConfiguration('max_z', default='2.0')

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('min_z', default_value=min_z),
        DeclareLaunchArgument('max_z', default_value=max_z),

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