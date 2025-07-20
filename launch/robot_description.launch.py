import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

robot_ns = ""

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('minim5robo_bridge'))
    urdf_file = os.path.join(pkg_path,
                            'urdf',
                            'minim5robo.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {'robot_description':  robot_desc}
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=robot_ns,
            parameters=[robot_description],
        ),
    ])