#Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    
    #Se corre el topico
    talker_node = Node(
        package='reto1',
        executable='suscriber',
        output='screen'
    )


    #Se ejecuta el graph
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
    )

    #Se ejecuta el plot
    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen',
        arguments=[{'args': '/ri_signal/data' '/ri_signal_reconstructed/data'}],  # Se especifican los tópicos
    )

    l_d = LaunchDescription([talker_node,rqt_graph_node, rqt_plot_node])
    return l_d
