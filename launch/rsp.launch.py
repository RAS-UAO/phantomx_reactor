#Librerias para Ubicar Archivos
import os
from ament_index_python import get_package_share_directory
#Librerias para Procesar Archivos
import xacro
#Librerias para Lanzar Nodos
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

pkg_file_path = os.path.join(get_package_share_directory("phantomx_reactor"))

use_sim_time = LaunchConfiguration("use_sim_time")
urdf_file_path = os.path.join(pkg_file_path, "urdf", "phantomx_reactor.urdf.xacro")
robot_description_file = xacro.process_file(urdf_file_path)

def generate_launch_description():
    rsp_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file.toxml(),
                "use_sim_time": use_sim_time
            }
        ]
    )

    nodes_to_run = [
            DeclareLaunchArgument(
                "use_sim_time", 
                default_value= "false", 
                description= "Use sim time if true"
            ),
            rsp_node
    ]
    return LaunchDescription(nodes_to_run)
