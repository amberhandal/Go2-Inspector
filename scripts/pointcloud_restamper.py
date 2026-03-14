#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

class PointCloudRestamper(Node):
    def __init__(self):
        super().__init__('pointcloud_restamper')

        # QoS for Go2 DDS compatibility
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        # Use relative topic names so launch file remaps work
        self.pub = self.create_publisher(PointCloud2, 'output', 10)
        self.sub = self.create_subscription(PointCloud2, 'input', self.callback, qos)

        self.get_logger().info(f'Restamping {self.sub.topic_name} -> {self.pub.topic_name}')
    
    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PointCloudRestamper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()