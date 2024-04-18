#Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    #Se declara la dirección del launchfile
    config = os.path.join(
        get_package_share_directory('reto2'),
        'config',
        'params.yaml'
        )
    

    odo_node = Node(
        package='reto1',
        executable='suscriber',
        output='screen'
    )
    #Se corre el signal generator
    talker_node = Node(
        package='reto2',
        executable='path_generator',
        output='screen',
        parameters = [config]
    )

    #Se corre el signal generator
    controller_node = Node(
        package='reto2',
        executable='controller',
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
        arguments=[],  # Se especifican los tópicos a graficar
    )
    #,rqt_graph_node, rqt_plot_node
    l_d = LaunchDescription([odo_node,talker_node, controller_node])
    return l_d
