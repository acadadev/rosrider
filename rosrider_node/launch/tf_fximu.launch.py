cd#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  base_link_to_imu_link_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_imu_link_fximu',
    arguments=['0.041', '0.0', '0.0496','0','0','0','base_link','imu_link']
  )

  ld = LaunchDescription()
  ld.add_action(base_link_to_imu_link_tf_node)

  return ld