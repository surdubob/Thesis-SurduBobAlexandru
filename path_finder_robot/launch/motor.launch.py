import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'path_finder_robot'


    arduino_http_comms = Node(
        package='path_finder_robot',
        executable='arduino_http_comms.py',
        output='screen'
    )


    # Run the node
    return LaunchDescription([
        arduino_http_comms
    ])