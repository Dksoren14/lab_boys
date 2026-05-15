#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanVolatileRelay(Node):
    def __init__(self):
        super().__init__('scan_volatile_relay')

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            sub_qos
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_volatile',
            pub_qos
        )

    def callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanVolatileRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()