"""Full launch: simulation, camera bridge, PX4 driver and joystick teleoperation.

The simulator paths and the Gazebo world (cats_world or tugbot_depot) are read
from launch_config.yaml. Generate it with scripts/generate_launch_config.py and
rebuild the package before launching.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from pose_estimator_pkg.launch_config import build_simulation_actions, load_sim_config

# Gazebo -> ROS 2 bridge arguments for the two drone cameras.
CAMERA_TOPICS = [
    "/pi_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/pi_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
    "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
]


def generate_launch_description():
    # Load settings from launch_config.yaml and start the Micro XRCE-DDS Agent,
    # Gazebo with the selected world and PX4 SITL.
    config = load_sim_config()
    simulation = build_simulation_actions(config)

    # Bridge the camera topics from Gazebo to ROS 2.
    cameras_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cameras_bridge",
        arguments=CAMERA_TOPICS,
        output="screen",
    )

    # Bridge between PX4 and ROS 2.
    px4_driver_node = Node(
        package="pose_estimator_pkg",
        executable="px4_driver",
        output="screen",
    )

    # Joystick (PS4 controller) driver.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": "/dev/input/js0",
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
    )

    # Teleoperation node: converts joystick commands into drone motion.
    move_drone_node = Node(
        package="pose_estimator_pkg",
        executable="move_drone",
        output="screen",
    )

    # Delays give Gazebo and PX4 time to start before the ROS 2 nodes.
    return LaunchDescription(simulation + [
        TimerAction(period=8.0, actions=[cameras_bridge]),
        TimerAction(period=10.0, actions=[px4_driver_node]),
        TimerAction(period=10.0, actions=[joy_node, move_drone_node]),
    ])
