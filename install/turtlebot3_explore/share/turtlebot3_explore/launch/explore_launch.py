# launch/explore_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener el directorio de recursos del paquete
    pkg_share = get_package_share_directory('turtlebot3_explore')
    
    # Construir la ruta completa al archivo de parámetros
    param_file = os.path.join(pkg_share, 'config', 'explore_params.yaml')
    
    # Verificar que el archivo de parámetros existe
    if not os.path.isfile(param_file):
        raise FileNotFoundError(f"Archivo de parámetros no encontrado: {param_file}")
    
    return LaunchDescription([
        Node(
            package='turtlebot3_explore',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[param_file]
        )
    ])
