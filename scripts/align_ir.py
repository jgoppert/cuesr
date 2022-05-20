#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

class AlignIR(Node):

    def __init__(self):
        super().__init__('align_ir')
        self.sub_ir = self.create_subscription(
            Image,
            '/thermal/camera/image_rect',
            self.ir_callback,
            10)
        self.sub_ir_info = self.create_subscription(
            CameraInfo,
            '/thermal/camera/camera_info',
            self.ir_info_callback,
            10)
        self.sub_depth = self.create_subscription(
            Image,
            '/camera/depth/image_rect',
            self.ir_callback,
            10)
        self.sub_ir_info = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.ir_info_callback,
            10)
        self.pub_aligned = self.create_publisher(Image, 'aligned', 10)


    def ir_callback(self, msg):
        self.get_logger().info('I heard ir')


def main(args=None):
    rclpy.init(args=args)

    align_ir = AlignIR()

    rclpy.spin(align_ir)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
