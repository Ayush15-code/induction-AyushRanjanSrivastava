#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from coordinate_follower.action import NavigateToPose

class NavigateClient(Node):
    def __init__(self):
        super().__init__('navigate_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_list = self.load_goals_from_file(
            "/home/ayush/induction-AyushRanjanSrivastava/coordinate_follower/coordinate_follower/goals.txt"
        )
        self.goal_index = 0

        self._action_client.wait_for_server()
        self.send_next_goal()

    def load_goals_from_file(self, filename):
        goals = []
        try:
            with open(filename, 'r') as f:
                for line in f:
                    if not line.strip():
                        continue
                    x_str, y_str = line.strip().split()
                    goals.append((float(x_str), float(y_str)))
        except Exception as e:
            self.get_logger().error(f"Failed to read goals from file: {e}")
        return goals

    def send_next_goal(self):
        if self.goal_index >= len(self.goal_list):
            self.get_logger().info("All goals sent and completed.")
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.x, goal_msg.y = self.goal_list[self.goal_index]
        self.get_logger().info(f'Sending goal #{self.goal_index + 1}: x={goal_msg.x}, y={goal_msg.y}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal #{self.goal_index + 1} rejected by server.')
            self.goal_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(f'Goal #{self.goal_index + 1} accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Goal #{self.goal_index + 1} succeeded!')
        else:
            self.get_logger().warn(f'Goal #{self.goal_index + 1} failed!')
        self.goal_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback — x: {fb.current_x:.2f}, y: {fb.current_y:.2f}, distance: {fb.distance_to_goal:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigateClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        

if __name__ == '__main__':
    main()
