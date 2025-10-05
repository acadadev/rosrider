import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

ROBOT_MODEL = os.environ['ROBOT_MODEL']

def generate_launch_description():

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # notice: we do not use any other config file then map mask and keeepout parameters

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mask_yaml_file = LaunchConfiguration('mask')

    default_mask_yaml_file = os.path.join(get_package_share_directory('rosrider_nav'), 'map', 'willow_mask.yaml')
    default_params_file = os.path.join(get_package_share_directory('rosrider_nav'), 'param', 'keepout_params.yaml')

    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_file, description='Full path to the ROS2 parameters file to use')
    declare_mask_yaml_file_cmd = DeclareLaunchArgument('mask', default_value=default_mask_yaml_file, description='Full path to filter mask yaml file to load')

    # TODO: test autostart. audit autostart on others. autostart out, others use it as default.

    # TODO: apply this pattern. if sim. on others
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes
            }])

    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            namespace=namespace,
            output='screen',
            parameters=[{configured_params}])

    start_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            parameters=[{configured_params}])

    keepout_script_path = os.path.join(get_package_share_directory('rosrider_nav'), 'scripts', 'service_call_keepout.py')

    keepout_launcher = ExecuteProcess(
        cmd=['/usr/bin/env', 'python3', keepout_script_path],
        name='service_call_keepout',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mask_yaml_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    ld.add_action(keepout_launcher)

    return ld