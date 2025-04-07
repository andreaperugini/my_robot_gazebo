from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Percorso del file URDF
    urdf_file_name = 'provatm.urdf'
    urdf_path = os.path.join(get_package_share_directory('my_robot_gazebo'), 'urdf', urdf_file_name)

    # Controllo se il file esiste
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"File URDF non trovato: {urdf_path}")

    # Lettura del file URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        )
    ])


