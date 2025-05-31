#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.publisher_ = self.create_publisher(Int32, 'number', qos_profile)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_number)
        self.number = 1

    def publish_number(self):
        msg = Int32()
        msg.data = self.number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.number += 1


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
