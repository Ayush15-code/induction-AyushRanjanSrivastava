#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.send_velocity_command)
        self.get_logger().info('CircleMover node started.')

    def send_velocity_command(self):
        twist = Twist()
        twist.linear.x = 0.2     # Moderate forward speed
        twist.angular.z = 0.2    # Moderate rotation speed
        self.publisher_.publish(twist)
        self.get_logger().info('Published: linear=0.2, angular=0.2')

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('CircleMover node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
