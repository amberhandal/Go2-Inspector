#!/usr/bin/env python3
"""
Restamps depth image and camera_info with identical timestamps.

The depth_image_proc point_cloud_xyz_node uses image_transport CameraSubscriber
with ExactTime sync, so the depth image and camera_info MUST have identical
timestamps. Two separate restamper nodes can't guarantee this — this combined
node ensures both messages get the exact same stamp.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class DepthSyncRestamper(Node):
    def __init__(self):
        super().__init__('depth_sync_restamper')

        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, 'output_image', pub_qos)
        self.info_pub = self.create_publisher(CameraInfo, 'output_camera_info', pub_qos)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'input_image', self.image_callback, sub_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, 'input_camera_info', self.info_callback, sub_qos)

        # Cache latest camera_info to pair with each depth image
        self.latest_info = None
        self.msg_count = 0

        self.get_logger().info(
            f'Depth sync restamper: '
            f'{self.image_sub.topic_name} + {self.info_sub.topic_name} -> '
            f'{self.image_pub.topic_name} + {self.info_pub.topic_name}')

        self.create_timer(5.0, self.status_callback)

    def info_callback(self, msg):
        self.latest_info = msg

    def image_callback(self, msg):
        if self.latest_info is None:
            return

        # Stamp both with the exact same time
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        self.latest_info.header.stamp = now

        self.image_pub.publish(msg)
        self.info_pub.publish(self.latest_info)
        self.msg_count += 1

    def status_callback(self):
        self.get_logger().info(
            f'Restamped {self.msg_count} synchronized depth pairs')


def main(args=None):
    rclpy.init(args=args)
    node = DepthSyncRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
