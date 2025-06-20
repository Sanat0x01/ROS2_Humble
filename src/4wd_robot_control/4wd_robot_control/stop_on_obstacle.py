import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Obstacle stopper node started.")

    def scan_callback(self, msg):
        front_ranges = self.get_front_ranges(msg)
        if any(r < 0.5 for r in front_ranges if r > 0.01):
            self.get_logger().info("Obstacle within 0.5m → Stopping.")
            self.stop_robot()
        else:
            self.move_forward()

    def get_front_ranges(self, msg):
        center_index = len(msg.ranges) // 2
        window = 30
        start = center_index - window // 2
        end = center_index + window // 2
        return msg.ranges[start:end]

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

