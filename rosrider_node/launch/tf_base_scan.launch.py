#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  base_link_to_base_scan_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_scan_ld19',
    arguments=['-0.166','-0.006','0.0706','0','0','0','base_link','base_scan']
  )

  ld = LaunchDescription()
  ld.add_action(base_link_to_base_scan_tf_node)

  return ld