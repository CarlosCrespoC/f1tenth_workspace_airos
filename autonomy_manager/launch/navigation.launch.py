import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # --- 1. Argumento para Elegir el Algoritmo ---
    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='rpp',
        description='Algoritmo de navegación a usar: rpp, ftg, etc.'
    )

    # --- 2. Configuración Específica para RPP_node ---
    rpp_share = get_package_share_directory('RPP_node')
    rpp_params_path = os.path.join(rpp_share, 'config', 'rpp_params.yaml')
    rpp_waypoints_path = PathJoinSubstitution([
        rpp_share, 'waypoints', LaunchConfiguration('waypoints_csv', default='crespo.csv')
    ])

    rpp_node = Node(
        package='RPP_node',
        executable='rpp_node_executable',
        name='rpp_follower',
        output='screen',
        # --- REMAPPING CRÍTICO ---
        # Conecta la salida del algoritmo al gestor de autonomía.
        remappings=[
            ('/drive', '/algo_drive_raw')
        ],
        parameters=[
            rpp_params_path,
            {'waypoints_csv': rpp_waypoints_path}
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('algorithm'), "' == 'rpp'"])
        )
    )

    # --- 3. Nodo Gestor de Autonomía (siempre se lanza) ---
    autonomy_manager_node = Node(
        package='autonomy_manager',
        executable='autonomy_manager_node',
        name='autonomy_manager',
        output='screen'
    )

    # --- 4. Lista de Componentes a Lanzar ---
    return LaunchDescription([
        algorithm_arg,
        autonomy_manager_node,
        rpp_node
    ])
