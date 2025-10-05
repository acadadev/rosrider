#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set initial pose (optional but recommended)
    '''
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw
    navigator.setInitialPose(initial_pose)
    '''

    # Wait for Nav2 to become active
    # navigator.waitUntilNav2Active()

    # Define goals
    goals = [
        {'x': 0.0, 'y': 0.0, 'yaw': -1.57},
        {'x': -4.0, 'y': -1.0, 'yaw': 1.57},
    ]

    for i, goal in enumerate(goals):
        print(f"Going to point {i + 1}: {goal}")

        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal['x']
        goal_pose.pose.position.y = goal['y']
        goal_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, goal['yaw'])
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        # Send goal
        navigator.goToPose(goal_pose)

        # Wait for result
        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        print(f"Result for point {i + 1}: {result}")

    print("Mission complete!")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
