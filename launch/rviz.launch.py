#Librerias para Ubicar Archivos
import os
from ament_index_python import get_package_share_directory
#Librerias para Lanzar Nodos
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

pkg_file_path = os.path.join(get_package_share_directory("phantomx_reactor"))
rsp_launch_file_path = os.path.join(pkg_file_path, "launch", "rsp.launch.py")
rviz_config_file_path = os.path.join(pkg_file_path, "config", "visualization.rviz")

def generate_launch_description():
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_file_path]),
        launch_arguments= {
            'use_sim_time': 'true'
        }.items()
    )
    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_config_file_path]
    )

    jsp_gui_node = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui",
    )

    nodes_to_run = [rsp_launch, rviz_cmd, jsp_gui_node]
    return LaunchDescription(nodes_to_run)
       
