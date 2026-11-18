#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.get_logger().info("waypoint_follower started")

        self.declare_parameter('csv_path', '/home/lucycv_u22/cats_ws/src/cats/cats/trajectories_dense/grid_0.csv')
        # self.declare_parameter('csv_path', '/home/lucycv_u22/cats_ws/src/cats/cats/trajectories/grid_0.csv')
        self.declare_parameter('kp_pos', 1.5)
        self.declare_parameter('kp_yaw', 0.9)
        self.declare_parameter('tolerance', 0.25)

        self.csv_path = self.get_parameter('csv_path').value
        self.kp_pos = self.get_parameter('kp_pos').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.tolerance = self.get_parameter('tolerance').value

        self.waypoints = self.load_csv(self.csv_path)
        self.index = 0

        self.pos_world = None
        self.yaw_world = 0.0

        self.create_subscription(PoseStamped, "/px4_driver/drone_pose", self.pose_callback,10)

        self.cmd_pub = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        self.declare_parameter("do_height_control", False)
        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value

        self.create_timer(0.05, self.control_loop)

    def load_csv(self, path):
        wps = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for r in reader:
                wps.append([
                    float(r['x']),
                    float(r['y']),
                    float(r['z']),
                    float(r['yaw'])
                ])
        return wps

    def pose_callback(self, msg):

        # posición
        x_world = msg.pose.position.y
        y_world = msg.pose.position.x
        z_world = -msg.pose.position.z

        self.pos_world = [x_world, y_world, z_world]

        # orientación
        q = msg.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        self.yaw_world = -yaw  

    def control_loop(self):
        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        if self.pos_world is None:
            return

        if self.index >= len(self.waypoints):
            return

        x_ref, y_ref, z_ref, yaw_ref = self.waypoints[self.index]
        x, y, z = self.pos_world

        # error en mundo
        ex = x_ref - x
        ey = y_ref - y
        ez = z_ref - z

        dist = math.sqrt(ex**2 + ey**2 + ez**2)

        if dist < self.tolerance:
            self.index += 1
            return

        # MUNDO -> BODY
        yaw = self.yaw_world

        vx_body =  math.cos(yaw)*ex + math.sin(yaw)*ey
        vy_body = -math.sin(yaw)*ex + math.cos(yaw)*ey
        vz_body = ez

        vx = self.kp_pos * vx_body
        vy = self.kp_pos * vy_body
        vz = self.kp_pos * vz_body

        vmax = 1.0
        vx = max(min(vx, vmax), -vmax)
        vy = max(min(vy, vmax), -vmax)
        vz = max(min(vz, vmax), -vmax)

        yaw_error = self.angle_diff(yaw_ref, yaw)
        wz = self.kp_yaw * yaw_error

        cmd = Twist()
        cmd.linear.x = vx   # forward
        cmd.linear.y = vy   # left
        cmd.linear.z = vz   # up
        cmd.angular.z = wz

        self.get_logger().info(f"x_ref={x_ref}, y_ref={y_ref}, z_ref={z_ref}, yaw_ref={yaw_ref}")
        self.get_logger().info(f"x={cmd.linear.x}, y={cmd.linear.y}, z={cmd.linear.z}, yaw={cmd.angular.z}")

        self.cmd_pub.publish(cmd)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()