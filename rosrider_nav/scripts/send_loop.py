#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import time


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to become active
    # navigator.waitUntilNav2Active()

    # Define your points
    goals_data = [
        { 'x': 0.0, 'y': 0.0, 'yaw': 0 },
        { 'x': 3.03, 'y': 1.04, 'yaw': 0 },
        { 'x': 3.08, 'y': 1.87, 'yaw': 0 },
    ]


    print("Starting looping mission. Press Ctrl+C to stop.")

    try:
        while True:  # Outer loop for continuous navigation
            for i, goal in enumerate(goals_data):
                # Create goal pose
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = goal['x']
                goal_pose.pose.position.y = goal['y']

                qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, goal['yaw'])
                goal_pose.pose.orientation.x = qx
                goal_pose.pose.orientation.y = qy
                goal_pose.pose.orientation.z = qz
                goal_pose.pose.orientation.w = qw

                print(f"Moving to: {goal}")
                navigator.goToPose(goal_pose)

                # Monitor progress
                while not navigator.isTaskComplete():
                    # Optional: Add feedback processing here
                    pass

                # Check result
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f"Reached point {i + 1} successfully.")
                elif result == TaskResult.CANCELED:
                    print(f"Task to point {i + 1} was canceled. Exiting...")
                    return
                elif result == TaskResult.FAILED:
                    print(f"Task to point {i + 1} failed.")

                # Small pause between points (optional)
                time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nMission interrupted by user. Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()