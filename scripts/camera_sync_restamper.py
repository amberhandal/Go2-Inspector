#!/usr/bin/env python3
"""
Restamps ALL camera data (RGB + depth) with identical timestamps.

RTAB-Map visual mode requires rgb/image and rgb/camera_info to have
matching timestamps for feature extraction. The depth/image also needs
a close timestamp for approx_sync. Separate restamper nodes can't
guarantee this - this combined node stamps all 4 camera topics with
the exact same clock value.

Trigger: depth image arrival (since RTAB-Map needs depth for 3D).
When depth arrives, all cached camera data is published with one
shared timestamp.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    DurabilityPolicy, HistoryPolicy
)
from sensor_msgs.msg import Image, CameraInfo


class CameraSyncRestamper(Node):
    def __init__(self):
        super().__init__('camera_sync_restamper')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers - synced outputs
        self.rgb_image_pub = self.create_publisher(
            Image, 'rgb_out/image', qos)
        self.rgb_info_pub = self.create_publisher(
            CameraInfo, 'rgb_out/camera_info', qos)
        self.depth_image_pub = self.create_publisher(
            Image, 'depth_out/image', qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, 'depth_out/camera_info', qos)

        # Subscribers - raw inputs
        self.rgb_image_sub = self.create_subscription(
            Image, 'rgb_in/image', self.rgb_image_cb, qos)
        self.rgb_info_sub = self.create_subscription(
            CameraInfo, 'rgb_in/camera_info',
            self.rgb_info_cb, qos)
        self.depth_image_sub = self.create_subscription(
            Image, 'depth_in/image', self.depth_image_cb, qos)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, 'depth_in/camera_info',
            self.depth_info_cb, qos)

        # Cache latest messages
        self.latest_rgb_image = None
        self.latest_rgb_info = None
        self.latest_depth_info = None
        self.sync_count = 0

        self.get_logger().info(
            'Camera sync restamper ready. '
            f'RGB: {self.rgb_image_sub.topic_name} -> '
            f'{self.rgb_image_pub.topic_name}, '
            f'Depth: {self.depth_image_sub.topic_name} -> '
            f'{self.depth_image_pub.topic_name}')

        self.create_timer(5.0, self.status_cb)

    def rgb_image_cb(self, msg):
        self.latest_rgb_image = msg

    def rgb_info_cb(self, msg):
        self.latest_rgb_info = msg

    def depth_info_cb(self, msg):
        self.latest_depth_info = msg

    def depth_image_cb(self, msg):
        """Trigger: on each depth image, publish all with same stamp."""
        if self.latest_depth_info is None:
            return

        now = self.get_clock().now().to_msg()

        # Depth pair (always publish)
        msg.header.stamp = now
        self.latest_depth_info.header.stamp = now
        self.depth_image_pub.publish(msg)
        self.depth_info_pub.publish(self.latest_depth_info)

        # RGB pair (publish if available, with SAME stamp)
        if self.latest_rgb_image is not None:
            self.latest_rgb_image.header.stamp = now
            self.rgb_image_pub.publish(self.latest_rgb_image)

        if self.latest_rgb_info is not None:
            self.latest_rgb_info.header.stamp = now
            self.rgb_info_pub.publish(self.latest_rgb_info)

        self.sync_count += 1

    def status_cb(self):
        has_rgb = self.latest_rgb_image is not None
        has_info = self.latest_rgb_info is not None
        self.get_logger().info(
            f'Published {self.sync_count} synced camera sets '
            f'(RGB: {"ok" if has_rgb else "waiting"}, '
            f'info: {"ok" if has_info else "waiting"})')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSyncRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
