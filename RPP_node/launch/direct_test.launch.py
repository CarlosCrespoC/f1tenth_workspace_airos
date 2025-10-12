import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
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
        # --- REMAPPING DIRECTO ---
        # Conecta la salida del nodo directamente al driver del VESC,
        # saltándose completamente el MUX y el gestor de autonomía.
        remappings=[
            ('/drive', '/ackermann_cmd')
        ],
        parameters=[
            rpp_params_path,
            {'waypoints_csv': rpp_waypoints_path}
        ]
    )

    return LaunchDescription([rpp_node])