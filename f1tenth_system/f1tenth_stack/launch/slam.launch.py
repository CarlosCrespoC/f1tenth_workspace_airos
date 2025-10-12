import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta a tu archivo de configuración YAML personalizado
    params_file = os.path.join(
        get_package_share_directory('f1tenth_stack'), # Nombre de tu paquete
        'config',
        'slam_toolbox.yaml' # Nombre de tu archivo de config
    )

    # Declaración del nodo de SLAM Toolbox
    # ROS 2 buscará 'slam_toolbox' y lo encontrará en /opt/ros/foxy/
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        start_async_slam_toolbox_node
    ])
