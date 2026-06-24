#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class AprilTagOverlay(Node):
    def __init__(self):
        super().__init__('apriltag_overlay')
        self.bridge = CvBridge()
        self.latest_image = None

        # Declare the parameter with a default value
        self.declare_parameter('image_topic', '/camera/image_raw')

        # Retrieve the parameter value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.get_logger().info(f'Subscribing to image topic: {image_topic}')

        # Use the parameter variable for the subscription
        self.image_sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.detections_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_cb, 10)
        self.image_pub = self.create_publisher(Image, '/apriltag_overlay', 10)

    def image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def detections_cb(self, msg):
        if self.latest_image is None:
            return

        overlay = self.latest_image.copy()

        for detection in msg.detections:
            # 1. Handle Corners & ID Placement
            if hasattr(detection, 'corners') and len(detection.corners) == 4:

                corners = np.array([[int(p.x), int(p.y)] for p in detection.corners], dtype=np.int32)

                # Draw the green bounding box around the tag
                cv2.polylines(overlay, [corners], True, (0, 255, 0), 2)

                # Modification 1: Place ID text precisely at the top-left corner (corners[0])
                top_left_x, top_left_y = corners[0][0], corners[0][1]
                cv2.putText(overlay, f"ID {detection.id}", (top_left_x, top_left_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Modification 2: Calculate Center & 2D Orientation from corners
                # Center is the average of all 4 corners
                center_x = int(np.mean(corners[:, 0]))
                center_y = int(np.mean(corners[:, 1]))

                # Heading vector can be derived from the bottom-to-top or left-to-right midpoint.
                # Let's use the vector from the left-side midpoint to the right-side midpoint for standard horizontal heading.
                mid_left_x = (corners[0][0] + corners[3][0]) / 2
                mid_left_y = (corners[0][1] + corners[3][1]) / 2
                mid_right_x = (corners[1][0] + corners[2][0]) / 2
                mid_right_y = (corners[1][1] + corners[2][1]) / 2

                # Calculate angle (yaw) in pixel space
                yaw = math.atan2(mid_right_y - mid_left_y, mid_right_x - mid_left_x)

                # Set dynamic length for the orientation arrow proportional to tag size
                arrow_length = int(np.linalg.norm(corners[0] - corners[1]) * 0.6)
                end_x = int(center_x + arrow_length * math.cos(yaw))
                end_y = int(center_y + arrow_length * math.sin(yaw))

                # Draw center anchor point and orientation arrow
                cv2.circle(overlay, (center_x, center_y), 4, (0, 0, 255), -1)
                cv2.arrowedLine(overlay, (center_x, center_y), (end_x, end_y), (0, 0, 255), 2, tipLength=0.3)

            # Fallback if corners are missing but ID exists
            elif hasattr(detection, 'id'):
                cv2.putText(overlay, f"ID: {detection.id}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        self.image_pub.publish(overlay_msg)


if __name__ == '__main__':
    rclpy.init()
    node = AprilTagOverlay()
    rclpy.spin(node)