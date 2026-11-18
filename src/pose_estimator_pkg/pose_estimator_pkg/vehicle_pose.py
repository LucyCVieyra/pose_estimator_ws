#!/usr/bin/env python3
import rclpy
import time
import csv
import os
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped

class VehiclePoseNode(Node):
    def __init__(self):
        super().__init__('vehicle_pose')
        self.get_logger().info('Nodo vehicle_pose iniciado')

        # QoS profile configuration
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(PoseStamped, "/px4_driver/drone_pose", self.update_pose, 10)

    def update_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll, pitch, yaw = self.euler_from_quaternion(qx, qy, qz, qw)

        self.get_logger().info(f"x={x}, y={y}, z={z}")
        # self.get_logger().info(f"roll={roll}, pitch={pitch}, yaw={yaw}")

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

def main(args=None):
    rclpy.init(args=args)
    node = VehiclePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()