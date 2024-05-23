#Importaciones necesarias
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    #Se declara la dirección del launchfile
    config = os.path.join(
        get_package_share_directory('reto_final'),
        'config',
        'params.yaml'
        )
    

    odometry_node = Node(
        package='reto_final',
        executable='odometry',
        output='screen'
    )

    error_line = Node(
        package='reto_final',
        executable='error_line',
        output='screen'
    )

    controller = Node(
        package='reto_final',
        executable='controller',
        output='screen'
    )

    # #Se ejecuta el graph
    # rqt_graph_node = Node(
    #     package='rqt_graph',
    #     executable='rqt_graph',
    #     output='screen',
    # )

    # #Se ejecuta el plot
    # rqt_plot_node = Node(
    #     package='rqt_plot',
    #     executable='rqt_plot',
    #     output='screen',
    #     arguments=[],  # Se especifican los tópicos a graficar
    # )
    
    
    l_d = LaunchDescription([odometry_node,error_line,controller])
    return l_d
