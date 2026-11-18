#!/usr/bin/env python3
import rclpy
import time
import cv2
import csv
import os
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from cv_bridge import CvBridge


class SavePosImageNode(Node):
    def __init__(self):
        super().__init__('save_pose_image')
        self.get_logger().info('Nodo save_pose_image iniciado')

        # QoS para los tópicos de PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Configuración general
        self.bridge = CvBridge()
        self.ts = 0.05  # 50 ms (20 Hz)
        self.count = 0            # contador de imágenes
        self.csv_filename = 'drone_data.csv'

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Crear carpeta de imágenes si no existe
        self.image_folder = 'images'
        os.makedirs(self.image_folder, exist_ok=True)

        # Crear el CSV con encabezado si no existe
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'Tiempo',      # timestamp (string)
                    'x', 'y', 'z', # posición
                    'q0', 'q1', 'q2', 'q3',  # cuaternión
                    'imagen'       # nombre de archivo de la imagen
                ])

        # Variables para almacenar el último mensaje recibido
        self.latest_image = None
        self.latest_pose = None
        self.latest_attitude = None

        # Suscripciones
        self.position_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.position_callback, qos_profile)

        self.attitude_sub = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile)

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Timer para guardar datos cada 0.05 s
        self.timer = self.create_timer(self.ts, self.timer_callback)

    # Callbacks de suscripción: solo guardan el último mensaje
    def position_callback(self, msg):
        self.latest_pose = msg

    def attitude_callback(self, msg):
        self.latest_attitude = msg

    def image_callback(self, msg):
        self.latest_image = msg

    def timer_callback(self):
        # Esperar hasta tener TODOS los datos
        if self.latest_image is None or self.latest_pose is None or self.latest_attitude is None:
            return

        current_time = time.time()
        timestamp_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))

        # Posición (en el frame en el que PX4 publica)
        pose_x = float(self.latest_pose.y)
        pose_y = float(self.latest_pose.x)
        pose_z = -float(self.latest_pose.z)

        # Orientación (cuaternión)
        pose_q0 = float(self.latest_attitude.q[0])
        pose_q1 = float(self.latest_attitude.q[1])
        pose_q2 = float(self.latest_attitude.q[2])
        pose_q3 = float(self.latest_attitude.q[3])

        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(pose_q1, pose_q2, pose_q3, pose_q0)

        # Guardar imagen con contador
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error al convertir imagen: {e}")
            return

        image_filename = f"image_{self.count:06d}.png"  
        image_path = os.path.join(self.image_folder, image_filename)
        cv2.imwrite(image_path, frame)

        # Guardar datos en CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp_str,
                pose_x, pose_y, pose_z,
                pose_q0, pose_q1, pose_q2, pose_q3,
                image_filename
            ])

        self.get_logger().info(
            f"[{self.count:06d}] Guardado: {image_filename} | "
            f"x={pose_x:.2f}, y={pose_y:.2f}, z={pose_z:.2f}, "
            f"roll={self.roll:.2f}, pitch={self.pitch:.2f}, yaw={self.yaw:.2f}"
        )

        # Incrementar contador
        self.count += 1

    def euler_from_quaternion(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


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
