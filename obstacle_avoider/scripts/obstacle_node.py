#!/usr/bin/env python3




import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Velocity message reused
        self.twist = Twist()

    def scan_callback(self, msg):
        # Focus on front range: typically 0° ± 10°
        front_ranges = list(msg.ranges[0:10] + msg.ranges[-10:])
        front_ranges = [r for r in front_ranges if r > 0.0]  # remove invalid readings (0.0)

        if not front_ranges:
            self.get_logger().warn("No valid LaserScan readings!")
            return

        min_distance = min(front_ranges)

        if min_distance > 0.5:
            # No obstacle — move forward
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
            self.get_logger().info("Moving forward")
        else:
            # Obstacle ahead — rotate right
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5
            self.get_logger().info("Obstacle detected! Turning right")

        self.cmd_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
