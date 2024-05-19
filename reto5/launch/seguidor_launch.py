#Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    controller_node = Node(
        package='reto5',
        executable='controller',
        output='screen'
    )

    error_line_node = Node(
        package='reto5',
        executable='error_line',
        output='screen'
    )

    #Se ejecuta el graph
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
    )
    
    
    l_d = LaunchDescription([controller_node,error_line_node])
    return l_d
