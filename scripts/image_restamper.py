#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class ImageRestamper(Node):
    def __init__(self):
        super().__init__('image_restamper')

        # Try RELIABLE first (Go2 RealSense publishes RELIABLE for some topics)
        # RELIABLE subscriber can receive from both RELIABLE and BEST_EFFORT publishers
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # For publishing, use compatible settings with RViz/RTAB-Map
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.pub = self.create_publisher(Image, 'output', pub_qos)
        self.sub = self.create_subscription(Image, 'input', self.callback, sub_qos)

        self.msg_count = 0
        self.get_logger().info(f'Restamping {self.sub.topic_name} -> {self.pub.topic_name}')
        
        self.create_timer(5.0, self.status_callback)

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.msg_count += 1

    def status_callback(self):
        self.get_logger().info(f'Received and restamped {self.msg_count} images so far')


def main(args=None):
    rclpy.init(args=args)
    node = ImageRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()