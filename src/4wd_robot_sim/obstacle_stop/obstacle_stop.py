import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')

        self.stop_threshold = 1.0  # Stop if obstacle is closer than 1 meter

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar_scan',
            self.scan_callback,
            qos_profile
        )
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            qos_profile
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )

        self.min_front_distance = float('inf')
        self.get_logger().info(" Obstacle Stopper Node Started (Stop threshold = 1.0 m)")

    def scan_callback(self, msg):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            self.get_logger().warn("LIDAR data is empty!")
            self.min_front_distance = float('inf')
            return

        center_index = num_ranges // 2
        window = min(15, center_index)  # Ensure window does not exceed index

        front_ranges = msg.ranges[center_index - window : center_index + window]
        valid_ranges = [r for r in front_ranges if 0.05 < r < float('inf')]

        if valid_ranges:
            self.min_front_distance = min(valid_ranges)
            self.get_logger().info(f"Min front distance: {self.min_front_distance:.2f} m")
        else:
            self.min_front_distance = float('inf')
            self.get_logger().warn("No valid LIDAR ranges in front arc")

    def cmd_callback(self, msg):
        # Log incoming command
        self.get_logger().info(f" Received cmd: linear.x = {msg.linear.x:.2f}, angular.z = {msg.angular.z:.2f}")

        if self.min_front_distance < self.stop_threshold and msg.linear.x > 0.0:
            self.get_logger().warn(" Obstacle too close! Blocking forward motion.")
            stop_msg = Twist()  # All zeros
            self.cmd_pub.publish(stop_msg)
        else:
            self.get_logger().info("Path clear. Forwarding command.")
            self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
