import os

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("minim5robo_bridge")
    list = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.038",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "0.0",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "imu_frame",
            ],
            # parameters=[{'use_sim_time': True}]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "-0.0555",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "1.57079632679",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "laser_frame",
            ],
            # parameters=[{'use_sim_time': True}]
        ),
    ]

    return LaunchDescription(list)
