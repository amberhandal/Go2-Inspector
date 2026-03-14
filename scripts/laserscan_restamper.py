#!/usr/bin/env python3
"""
LaserScan Restamper Node

Re-stamps LaserScan messages with current ROS time to fix timestamp
synchronization issues between Go2 robot and PC.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserScanRestamper(Node):
    def __init__(self):
        super().__init__('laserscan_restamper')
        
        # QoS for sensor data compatibility
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        
        # Use remapped topics (input/output allow launch file configuration)
        self.pub = self.create_publisher(LaserScan, 'output', 10)
        self.sub = self.create_subscription(LaserScan, 'input', self.callback, qos)
        
        self.msg_count = 0
        self.get_logger().info('LaserScan Restamper started')
        self.get_logger().info(f'  Subscribing to: {self.sub.topic_name}')
        self.get_logger().info(f'  Publishing to: {self.pub.topic_name}')
    
    def callback(self, msg):
        # Update timestamp to current time
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        
        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(f'Restamped {self.msg_count} LaserScan messages')


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanRestamper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()