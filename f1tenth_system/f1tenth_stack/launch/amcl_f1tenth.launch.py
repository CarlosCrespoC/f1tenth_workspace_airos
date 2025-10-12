# amcl_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Asegúrate de que 'f1tenth_stack' es el nombre correcto de tu paquete
    pkg_share = get_package_share_directory('f1tenth_stack') 
    
    # Declarar el argumento para la ruta del mapa
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'mapa_lab.yaml'), # <-- RUTA A TU MAPA
        description='Full path to map file to load'
    )

    # Lanzar el servidor de mapas
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}]
    )

    # Lanzar AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': False,        # <-- Cambia a True si usas simulación
                     'alpha1': 0.1,
                     'alpha2': 0.1,
                     'alpha3': 0.1,
                     'alpha4': 0.1,
                     'base_frame_id': 'base_link',   # <-- Frame base de tu robot
                     'odom_frame_id': 'odom',        # <-- Frame de odometría
                     'global_frame_id': 'map',       # <-- ¡COMA AÑADIDA AQUÍ!
                     'max_particles': 2000,
                     'min_particles': 500,
                     'update_min_d': 0.05,
                     'update_min_a': 0.15,
                     'laser_max_beams': 360
                    }]
    )

    # Lanzar el gestor de ciclo de vida para activar los nodos
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])