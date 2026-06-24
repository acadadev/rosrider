#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class FixPMatrixRectNode(Node):
    def __init__(self):
        super().__init__('fix_p_matrix_rect')
        self.get_logger().info("Starting Rectified Camera Info Publisher bound to Rectified Image Stamps")

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_rect',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            CameraInfo,
            '/camera_info',
            10
        )

    def listener_callback(self, img_msg):

        out_msg = CameraInfo()

        out_msg.header.stamp = img_msg.header.stamp
        out_msg.header.frame_id = img_msg.header.frame_id

        out_msg.width = 640
        out_msg.height = 400

        out_msg.distortion_model = "plumb_bob"

        # The rectified image has zero geometric distortion left over
        out_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Base zoom factor used to prevent image_proc from cropping the 160-deg FOV edges
        f_rectified = 110.0

        # Overwrite K to match the virtual zoom. This forces the AprilTag solver
        # to calculate an accurate 0.25m scale multiplier instead of 1.0m.
        out_msg.k = [
            f_rectified, 0.0, 320.0,
            0.0, f_rectified, 200.0,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix defaults to identity
        out_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Mirror the projection matrix
        out_msg.p = [
            f_rectified, 0.0, 320.0, 0.0,
            0.0, f_rectified, 200.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixPMatrixRectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()