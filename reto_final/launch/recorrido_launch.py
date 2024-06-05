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

    sign_information = Node(
        package='reto_final',
        executable='sign_information',
        output='screen'
    )
    
    
    l_d = LaunchDescription([error_line,odometry_node,controller])
    return l_d
