import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'path_finder_robot'
    file_subpath = 'urdf/path_finder.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    # )

    # node_joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen',
    # )

    # node_laser_scan_matcher = Node(
    #     package='ros2_laser_scan_matcher',
    #     executable='laser_scan_matcher',
    #     output='screen',
    # )

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar'),
                'launch/ydlidar_launch.py'))
    )

    node_movement_controller = Node(
        package='path_finder_robot',
        executable='motor_controller_node.py',
        output='screen',
    )

    # launch_include2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('path_finder_web'),
    #             'launch/example.launch.py'))
    # )


    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        node_laser_scan_matcher,
        node_movement_controller,
        launch_include,
        launch_include2
    ])