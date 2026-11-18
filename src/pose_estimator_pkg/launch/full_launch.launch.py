from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from pose_estimator_pkg.launch_config import get_launch_paths

def generate_launch_description():
    sim_repo, px4_repo, world_path = get_launch_paths()
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", str(world_path)],
        cwd=str(sim_repo),
        output="screen"
    )

    px4_sitl = ExecuteProcess(
        cmd=["/bin/bash", "-lc", "HEADLESS=1 make px4_sitl gz_x500"],
        cwd=str(px4_repo),
        output="screen"
    )

    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "--port", "8888"],
        output="screen"
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/pi_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/pi_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen"
    )

    px4_driver_node = Node(
        package="pose_estimator_pkg",
        executable="px4_driver",
        output="screen"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": "/dev/input/js0",  
            "deadzone": 0.05,
            "autorepeat_rate": 20.0
        }]
    )

    move_drone_node = Node(
        package="pose_estimator_pkg",
        executable="move_drone",
        output="screen"
    )

    return LaunchDescription([
        microxrce_agent,
        gz_sim,
        px4_sitl,
        TimerAction(period=8.0, actions=[gz_bridge]),
        TimerAction(period=10.0, actions=[px4_driver_node]),
        TimerAction(period=10.0, actions=[joy_node, move_drone_node]),
    ])
