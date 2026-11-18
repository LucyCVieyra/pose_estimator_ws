#!/usr/bin/env python3
import rclpy
import time
import cv2
import csv
import os
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class SavePosImageNode(Node):
    def __init__(self):
        super().__init__('save_pose_image')
        self.get_logger().info('Nodo save_pose_image iniciado')

        # Configuración general
        self.bridge = CvBridge()
        self.ts = 0.1
        self.count = 0
        self.init_number = 0
        self.dataset_size = 1500
        self.csv_filename = 'drone_data.csv'

        # Carpeta de imágenes
        self.image_folder = 'images'
        os.makedirs(self.image_folder, exist_ok=True)

        # CSV
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'Tiempo',
                    'x', 'y', 'z',
                    'roll', 'pitch', 'yaw',
                    'imagen'
                ])

        # Variables
        self.latest_image = None
        self.latest_pose = None
        self.x0 = 6
        self.y0 = 0

        # Subscripciones
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/px4_driver/drone_pose",
            self.pose_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Timer
        self.timer = self.create_timer(self.ts, self.timer_callback)

    def pose_callback(self, msg):
        self.latest_pose = msg

    def image_callback(self, msg):
        self.latest_image = msg

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
        if self.latest_image is None or self.latest_pose is None:
            return

        current_time = time.time()
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))

        # Posición
        pose = self.latest_pose.pose

        x = pose.position.y + self.x0
        y = pose.position.x + self.y0
        z = -pose.position.z

        # Orientación → Euler
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        roll, pitch, raw_yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        raw_yaw_deg = numpy.rad2deg(raw_yaw)
        yaw = numpy.pi/2 - raw_yaw
        yaw = numpy.arctan2(numpy.sin(yaw), numpy.cos(yaw))  # Normalizar entre -pi y pi
        yaw_deg = numpy.rad2deg(yaw)

        # Imagen
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error al convertir imagen: {e}")
            return

        if self.count <= self.dataset_size:
            count_name = self.count + self.init_number
            image_filename = f"image_{count_name:06d}.png"
            image_path = os.path.join(self.image_folder, image_filename)
            cv2.imwrite(image_path, frame)

            # CSV
            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    timestamp_str,
                    x, y, z,
                    roll, pitch, yaw,
                    image_filename
                ])
            self.get_logger().info(
                f"[{self.count:06d}] {image_filename} | "
                f"x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw_deg:.2f}"
            )
        else:
            self.get_logger().info("Se ha alcanzado el tamaño máximo del dataset.")
            self.timer.cancel()  # Detener el timer si se alcanza el tamaño máximo
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SavePosImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()