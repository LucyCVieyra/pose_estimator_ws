#!/usr/bin/env python3
"""
Nodo ROS2 que ejecuta el modelo PoseNetLight (entrenado con cats_pose_local_v02.py)
sobre las imágenes RGB del dron y publica la pose estimada en /nn_image_pose.

Arquitectura, normalización de posición y preprocesamiento de imagen replicados
exactamente del script de entrenamiento para garantizar consistencia train/inferencia.

Suscribe:
    /camera/image_raw   (sensor_msgs/Image)

Publica:
    /nn_image_pose      (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node

import torch
import torch.nn as nn
import numpy as np
from torchvision import transforms, models
from PIL import Image as PILImage

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


# ==============================================================================
# CONSTANTES DE NORMALIZACIÓN DE POSICIÓN
# Deben ser IDÉNTICAS a las usadas en cats_pose_local_v02.py durante el
# entrenamiento, o el denormalizado dará valores incorrectos.
# ==============================================================================
X_MIN, X_MAX = -1.25, 8.75
Y_MIN, Y_MAX = -8.0, 2.0
Z_MIN, Z_MAX = 0.134733498096466, 4.34491205215454

POS_MIN = torch.tensor([X_MIN, Y_MIN, Z_MIN], dtype=torch.float32)
POS_MAX = torch.tensor([X_MAX, Y_MAX, Z_MAX], dtype=torch.float32)
POS_RANGE = POS_MAX - POS_MIN

IMAGE_SIZE = 224


def denormalize_pos(pos_norm: torch.Tensor) -> torch.Tensor:
    """Igual a la función usada en entrenamiento."""
    return (pos_norm + 1) / 2 * POS_RANGE.to(pos_norm.device) + POS_MIN.to(pos_norm.device)


# ==============================================================================
# ARQUITECTURA DEL MODELO (idéntica a cats_pose_local_v02.py)
# ==============================================================================
class PoseNetLight(nn.Module):
    def __init__(self, pretrained=False):
        super().__init__()
        weights = models.ResNet18_Weights.DEFAULT if pretrained else None
        backbone = models.resnet18(weights=weights)
        self.backbone = nn.Sequential(*list(backbone.children())[:-1])
        self.backbone_out_dim = backbone.fc.in_features

        self.fc = nn.Sequential(
            nn.Linear(self.backbone_out_dim, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 7),
        )

    def forward(self, x):
        x = self.backbone(x)
        x = torch.flatten(x, 1)
        return self.fc(x)


# Mismo preprocesamiento que eval_transform en el entrenamiento
eval_transform = transforms.Compose([
    transforms.Resize((IMAGE_SIZE, IMAGE_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])


class NNPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('nn_pose_estimator')
        self.get_logger().info('Nodo nn_pose_estimator iniciado')

        # ------------------------------------------------------------------
        # CONFIGURACIÓN - AJUSTA ESTO
        # ------------------------------------------------------------------
        # Ruta al checkpoint entrenado (posenet_light_best_local.pth o _last_)
        self.model_path = r"/home/lucycv_u22/cats_ws/src/cats/best_6.pth"
        self.frame_id = 'map'  # frame en el que se publica la pose estimada
        # ------------------------------------------------------------------

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Usando device: {self.device}')

        self.model = self.load_model(self.model_path)
        self.transform = eval_transform

        self.latest_image = None

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/nn_image_pose', 10)

    # ----------------------------------------------------------------------
    # Carga del modelo
    # ----------------------------------------------------------------------
    def load_model(self, path):
        checkpoint = torch.load(path, map_location=self.device)
        model = PoseNetLight(pretrained=False)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.to(self.device)
        model.eval()
        self.get_logger().info(f'Modelo cargado desde: {path}')
        return model

    # ----------------------------------------------------------------------
    # Callback de imagen -> inferencia inmediata
    # ----------------------------------------------------------------------
    def image_callback(self, msg):
        self.latest_image = msg
        self.run_inference()

    def run_inference(self):
        if self.latest_image is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error al convertir imagen: {e}")
            return

        # BGR (OpenCV) -> RGB (PIL/torchvision), igual que en entrenamiento
        frame_rgb = frame[:, :, ::-1]
        pil_img = PILImage.fromarray(frame_rgb.copy())

        input_tensor = self.transform(pil_img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(input_tensor)

        pos_norm = output[:, :3]
        quat = output[:, 3:]

        # Denormalizar posición (igual que en entrenamiento)
        pos = denormalize_pos(pos_norm).squeeze(0).cpu().numpy()

        # Normalizar quaternion a norma unitaria
        quat = quat / (torch.norm(quat, p=2, dim=1, keepdim=True) + 1e-8)
        quat = quat.squeeze(0).cpu().numpy()

        x, y, z = pos
        # dataset.csv: q0=x, q1=y, q2=z, q3=w -> coincide con geometry_msgs/Quaternion
        qx, qy, qz, qw = quat

        self.publish_pose(x, y, z, qx, qy, qz, qw)

    # ----------------------------------------------------------------------
    # Publicación
    # ----------------------------------------------------------------------
    def publish_pose(self, x, y, z, qx, qy, qz, qw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.pose_pub.publish(msg)

        self.get_logger().info(
            f"Pose estimada -> x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NNPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()