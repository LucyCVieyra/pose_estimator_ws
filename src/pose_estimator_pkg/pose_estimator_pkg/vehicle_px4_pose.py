#!/usr/bin/env python3
import rclpy
import os
import numpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleAttitude


class VehiclePX4PoseNode(Node):
    def __init__(self):
        super().__init__('vehicle_px4_pose')
        self.get_logger().info('Nodo vehicle_px4_pose started')

        # QoS para los tópicos de PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.latest_attitude = None
        self.ts = 0.05

        # Suscripciones
        self.attitude_sub = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_px4_pose", self.attitude_callback, qos_profile)

        # Timer para guardar datos cada 0.05 s
        self.timer = self.create_timer(self.ts, self.timer_callback)

    def attitude_callback(self, msg):
        self.latest_attitude = msg

    def timer_callback(self):
        # Esperar hasta tener TODOS los datos
        if self.latest_attitude is None:
            return

        current_time = time.time()
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))

        # Orientación (cuaternión)
        roll, pitch, yaw = self.euler_from_quaternion(self.latest_attitude)

        self.get_logger().info(f"roll={roll}, pitch={pitch}, yaw={yaw}")
    
    def euler_from_quaternion(self, attitude):
        x = attitude.q[0]
        y = attitude.q[1]
        z = attitude.q[2]
        w = attitude.q[3]

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
    node = VehicleAttitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
