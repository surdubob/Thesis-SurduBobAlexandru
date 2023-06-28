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
    file_subpath = 'urdf/path_finder.urdf.xacro'


    # Use xacro to process the file
    # xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()


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



    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )



    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'my_controllers.yaml')

    arduino_http_comms = Node(
        package='path_finder_robot',
        executable='arduino_http_comms.py',
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=["diff_cont"]
    )

    delayed_diffdrive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=['--params-file', '/home/alex/ros2_ws/src/path_finder_robot/config/twist_mux.yaml'],
        remappings=[
            ("cmd_vel_out", "diff_cont/cmd_vel_unstamped")
        ]
    )

    web_interface = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('path_finder_web'),'launch','web.launch.py'
                )])
    )

    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ydlidar'),'launch','ydlidar_launch.py'
                )])
    )

    camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'
                )])
    )


    # Run the node
    return LaunchDescription([
        rsp,
        arduino_http_comms,
        delayed_controller_manager,
        delayed_diffdrive_spawner,
        delayed_joint_broad_spawner,
        twist_mux,
        # web_interface,
        lidar,
        camera
    ])
