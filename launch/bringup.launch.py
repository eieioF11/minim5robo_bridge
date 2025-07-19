import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    pkg_dir = get_package_share_directory("minim5robo_bridge")
    declare_launch_slam = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Launch slam or not",
    )
    declare_launch_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch rviz or not",
    )
    launch_slam = LaunchConfiguration("slam")
    launch_rviz = LaunchConfiguration("rviz")
    list = [
        declare_launch_slam,
        declare_launch_rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_dir, "launch"), "/static_tf.launch.py"]
            ),
        ),
        Node(
            package='minim5robo_bridge',
            executable='minim5robo_bridge',
            namespace='',
            output="screen",
            respawn=True,
        ),
        # Node(
        #     package='manual_controller',
        #     executable='manual_controller',
        #     namespace='',
        #     output="screen",
        #     parameters=[os.path.join(pkg_dir, "config", "manual_controller_param.yaml")],
        #     respawn=True,
        # ),
        Node(
            package='joy',
            executable='joy_node',
            namespace='',
            output="screen",
            respawn=True,
        ),
        Node(
            package='twist_switcher',
            executable='twist_switcher',
            namespace='',
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "twist_switcher_param.yaml")],
            respawn=True,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_dir, "launch"), "/rviz.launch.py"]
            ),
            condition=IfCondition(launch_rviz) # ここでconditionがtrueのときに呼び出すように指定
        ),
    ]

    return LaunchDescription(list)
