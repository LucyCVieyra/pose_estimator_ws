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

class PosePubNode(Node):
    def __init__(self):
        super().__init__('pose_pub')
        self.get_logger().info('Nodo pose_pub iniciado')

        self.ts = 0.5

        self.latest_pose = None
        self.frame_id = 'map'
        self.x0 = 0.0
        self.y0 = 0.0

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/px4_driver/drone_pose",
            self.pose_callback,
            10
        )

        self.px4_pose_pub = self.create_publisher(PoseStamped, "/px4_pose", 10)

        # Timer
        self.timer = self.create_timer(self.ts, self.timer_callback)

    def pose_callback(self, msg):
        self.latest_pose = msg

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
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Posición
        pose = self.latest_pose.pose

        msg.pose.position.x = pose.position.y + self.x0
        msg.pose.position.y = pose.position.x + self.y0
        msg.pose.position.z = -pose.position.z

        # Orientación → Euler
        msg.pose.orientation.x = pose.orientation.x
        msg.pose.orientation.y = pose.orientation.y
        msg.pose.orientation.z = pose.orientation.z
        msg.pose.orientation.w = pose.orientation.w

        self.px4_pose_pub.publish(msg)

        self.get_logger().info(
            f"Pose PX4 -> x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PosePubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()