#!/usr/bin/env python3

import numpy as np
np.float = float  # Fix for deprecated np.float in some packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import math
import tf_transformations

class ArucoNavigator(Node):
    def __init__(self):
        super().__init__('aruco_navigator')
        self.get_logger().info("✅ ArUco Navigator Node Initialized")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.tag_size = 0.1778  # meters
        self.camera_matrix = np.array([[1696.8, 0, 960.5],
                                       [0, 1696.8, 540.5],
                                       [0, 0, 1]])
        self.dist_coeffs = np.zeros((4, 1))

        self.following = True
        self.turned = False
        self.rotating = False
        self.post_turn_moving = False
        self.current_yaw = 0.0
        self.target_yaw = None

        self.rotation_timer = self.create_timer(0.05, self.rotation_control)

    def image_callback(self, msg):
        if self.rotating or self.post_turn_moving:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info(f"🎯 Tags detected: {ids.flatten().tolist()}")
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.tag_size, self.camera_matrix, self.dist_coeffs
            )

            z = tvecs[0][0][2]
            self.get_logger().info(f"📏 Distance to tag: {z:.2f} meters")

            if z > 1.0 and self.following:
                self.get_logger().info("🚶 Moving forward towards tag...")
                self.move_forward()
            elif not self.turned:
                self.get_logger().info("🛑 Reached stopping distance, initiating rotation...")
                self.stop_moving()
                self.following = False
                tag_id = ids[0][0]
                self.get_logger().info(f"🆔 Detected Tag ID: {tag_id}")
                if tag_id == 0:
                    self.rotate_by_angle(-math.pi / 2)  # Right
                elif tag_id == 1:
                    self.rotate_by_angle(math.pi / 2)   # Left
        else:
            if self.following:
                self.get_logger().warn("🚫 No tags detected. Continuing forward.")
                self.move_forward()

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_yaw = yaw

    def rotation_control(self):
        if self.rotating and self.target_yaw is not None:
            angle_diff = self.angle_difference(self.current_yaw, self.target_yaw)
            self.get_logger().info(
                f"🧭 Rotating... Current Yaw: {math.degrees(self.current_yaw):.2f}, "
                f"Target: {math.degrees(self.target_yaw):.2f}, "
                f"Diff: {math.degrees(angle_diff):.2f}"
            )

            if abs(angle_diff) < 0.03:  # ~1.7 degrees
                self.get_logger().info("🛑 Target yaw reached.")
                self.stop_moving()
                self.rotating = False
                self.turned = True
                self.move_forward_short_distance()
            else:
                twist = Twist()
                twist.angular.z = 0.3 if angle_diff > 0 else -0.3
                self.cmd_vel_pub.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.15
        self.cmd_vel_pub.publish(twist)

    def stop_moving(self):
        self.cmd_vel_pub.publish(Twist())  # All zeros

    def move_forward_short_distance(self):
        self.post_turn_moving = True
        twist = Twist()
        twist.linear.x = 0.12
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("🚶 Moving forward slightly after turn...")
        self.create_timer(1.2, self.stop_after_post_turn)

    def stop_after_post_turn(self):
        if self.post_turn_moving:
            self.get_logger().info("🛑 Post-turn forward complete.")
            self.stop_moving()
            self.post_turn_moving = False
            self.following = True
            self.turned = False

    def rotate_by_angle(self, angle):
        self.rotating = True
        self.target_yaw = self.normalize_angle(self.current_yaw + angle)
        self.get_logger().info(f"{'🔄 Left' if angle > 0 else '🔁 Right'} turn started. Target yaw: {math.degrees(self.target_yaw):.2f}°")

    def quaternion_to_euler(self, x, y, z, w):
        return tf_transformations.euler_from_quaternion([x, y, z, w])

    def angle_difference(self, a, b):
        return (b - a + math.pi) % (2 * math.pi) - math.pi

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
