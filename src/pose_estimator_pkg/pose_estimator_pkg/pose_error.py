#!/usr/bin/env python3
import rclpy
import time
import cv2
import csv
import os
import numpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class PoseErrorNode(Node):
    def __init__(self):
        super().__init__('pose_error')
        self.get_logger().info('Nodo pose_error iniciado')

        self.ts = 1.0

        self.latest_pose_nn = None
        self.latest_pose_px4 = None
        self.frame_id = 'map'

        self.px4_sub = self.create_subscription(PoseStamped, "/px4_pose", self.px4_callback, 10)
        self.nn_sub = self.create_subscription(PoseStamped, "/nn_image_pose", self.nn_callback, 10)

        # Timer
        self.timer = self.create_timer(self.ts, self.timer_callback)

    def px4_callback(self, msg):
        self.latest_pose_px4 = msg

    def nn_callback(self, msg):
        self.latest_pose_nn = msg

    def euler_from_quaternion(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        current_time = time.time()
        pose_px4 = self.latest_pose_px4
        pose_nn = self.latest_pose_nn

        x_error = pose_nn.pose.position.x - pose_px4.pose.position.x
        y_error = pose_nn.pose.position.y - pose_px4.pose.position.y
        z_error = pose_nn.pose.position.z - pose_px4.pose.position.z

        roll_px4, pitch_px4, yaw_px4 = self.euler_from_quaternion(
            pose_px4.pose.orientation.x,
            pose_px4.pose.orientation.y,
            pose_px4.pose.orientation.z,
            pose_px4.pose.orientation.w
        )

        roll_nn, pitch_nn, yaw_nn = self.euler_from_quaternion(
            pose_nn.pose.orientation.x,
            pose_nn.pose.orientation.y,
            pose_nn.pose.orientation.z,
            pose_nn.pose.orientation.w
        )

        roll_error = roll_nn - roll_px4
        pitch_error = pitch_nn - pitch_px4
        yaw_error = yaw_nn - yaw_px4

        self.get_logger().info(f"Position errors: x={x_error:.4f}, y={y_error:.4f}, z={z_error:.4f}")
        # self.get_logger().info(f"Orientation errors: roll={roll_error:.4f}, pitch={pitch_error:.4f}, yaw={yaw_error:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()