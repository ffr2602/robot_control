import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'robot_control'  

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rsp_launch.py')])
    )

    imu_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','bno055_launch.py')])
    )

    control_params = os.path.join(get_package_share_directory(package_name),'config','control_params.yaml')
    controller = Node(
        package=package_name,
        executable='omni_controller.py',
        name='omni_controller',
        parameters=[control_params]
    )

    path_tracking = Node(
        package=package_name,
        executable='trajectory_tracking',
        name='path_tracking',
        parameters=[control_params]
    )

    node_path_generate = Node(
        package=package_name,
        executable='path_tracking.py',
        name='node_path_generate',
        parameters=[control_params]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/omni_cont/cmd_vel_unstamped')]
    )

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'rosbridge_websocket_launch.xml')])
    )

    return LaunchDescription([
        rsp,
        controller,
        imu_sensor,
        # twist_mux,
        path_tracking,
        node_path_generate,
        rosbridge
    ])

