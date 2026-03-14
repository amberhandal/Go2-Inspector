#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoRestamper(Node):
    def __init__(self):
        super().__init__('camera_info_restamper')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(CameraInfo, 'output', 10)
        self.sub = self.create_subscription(CameraInfo, 'input', self.callback, qos)

        self.get_logger().info(f'Restamping {self.sub.topic_name} -> {self.pub.topic_name}')

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
