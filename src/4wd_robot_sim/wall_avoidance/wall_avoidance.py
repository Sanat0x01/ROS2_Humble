import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallAvoider(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        
        # Publisher to send velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to get LIDAR data from the robot
        self.scan_sub = self.create_subscription(LaserScan, '/lidar_scan', self.lidar_callback, 10)
        
        self.twist = Twist()
        self.obstacle_threshold = 0.8  # meters
        self.front_distance = float('inf')

        # Call control loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.control_loop)

    def lidar_callback(self, msg):
        # Extract LIDAR front sector (30 readings centered)
        ranges = msg.ranges
        center_index = len(ranges) // 2
        front_ranges = ranges[center_index - 15:center_index + 15]

        # Filter out NaNs and infs
        valid_ranges = [r for r in front_ranges if r > 0.01 and r < 10.0]

        if valid_ranges:
            self.front_distance = min(valid_ranges)
        else:
            self.front_distance = float('inf')

    def control_loop(self):
        if self.front_distance > self.obstacle_threshold:
            # Move forward
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
        else:
            # Rotate in place
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5

        self.cmd_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

