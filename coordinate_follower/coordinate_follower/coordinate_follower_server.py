#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from coordinate_follower.action import NavigateToPose
import math

class NavigateServer(Node):
    def __init__(self):
        super().__init__('navigate_server')
        self.get_logger().info("Action Server started.")

        self._server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        rate = self.create_rate(10)  # 10 Hz

        self.get_logger().info(f"Received goal: x={goal.x}, y={goal.y}")

        # Wait for odometry to initialize
        self.get_logger().info("Waiting for odometry...")
        while (self.current_x == 0.0 and self.current_y == 0.0) and rclpy.ok():
            rate.sleep()
        self.get_logger().info("Odometry received. Starting goal execution.")

        max_steps = 600
        step = 0

        while rclpy.ok():
            dx = goal.x - self.current_x
            dy = goal.y - self.current_y
            distance = math.hypot(dx, dy)

            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.distance_to_goal = distance
            goal_handle.publish_feedback(feedback)

            if distance < 0.1:
                self.get_logger().info("Goal reached.")
                break

            # Orientation control
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            self.get_logger().info(
                f"angle_error: {math.degrees(angle_error):.2f}°, distance: {distance:.2f}"
            )

            twist = Twist()
            if abs(angle_error) > 0.3:
                twist.linear.x = 0.0
                twist.angular.z = max(min(0.6 * angle_error, 1.0), -1.0)
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0

            self.get_logger().info(
                f"Publishing cmd_vel — linear.x: {twist.linear.x:.2f}, angular.z: {twist.angular.z:.2f}"
            )

            self.cmd_pub.publish(twist)
            rate.sleep()

            step += 1
            if step > max_steps:
                self.get_logger().warn("Timeout: Failed to reach goal in time.")
                break

        self.cmd_pub.publish(Twist())

        if distance < 0.1:
            result.success = True
            goal_handle.succeed()
            self.get_logger().info("Goal succeeded!")
        else:
            result.success = False
            goal_handle.abort()
            self.get_logger().warn("Goal failed.")

        return result

def main(args=None):
    print("coordinate_follower_server.py main() running")
    rclpy.init(args=args)
    node = NavigateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
