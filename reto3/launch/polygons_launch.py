#Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    #Se declara la dirección del launchfile
    config = os.path.join(
        get_package_share_directory('reto3'),
        'config',
        'params.yaml'
        )
    

    odometry_node = Node(
        package='reto3',
        executable='odometry',
        output='screen'
    )

    path_generator_node = Node(
        package='reto3',
        executable='path_generator',
        output='screen'
    )

    controller_node = Node(
        package='reto3',
        executable='controller',
        output='screen',
        parameters = [config]
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
    
    
    l_d = LaunchDescription([odometry_node,path_generator_node,controller_node])
    return l_d
