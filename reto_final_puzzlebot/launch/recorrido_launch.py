#Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    # Se obtiene el nodo que calcula el error para el seguidor de línea
    error_line = Node(
        package='reto_final_puzzlebot',
        executable='error_line',
        output='screen'
    )
    
    # Se obtiene el nodo que calcula el control para el recorrido.
    controller = Node(
        package='reto_final_puzzlebot',
        executable='controller',
        output='screen'
    )
    
    l_d = LaunchDescription([error_line,controller])
    return l_d
