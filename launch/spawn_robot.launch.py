from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix



def generate_launch_description():

    package_name = 'my_robot_gazebo'
    package_path = get_package_share_directory(package_name)
    ros2_control_config_path = os.path.join(package_path, 'config', 'ros2_control.yaml')


    pkg_gazebo_ros = os.popen("ros2 pkg prefix gazebo_ros").read().strip()
    gazebo_launch = os.path.join(pkg_gazebo_ros, "share", "gazebo_ros", "launch", "gazebo.launch.py")

    pkg_share_path = os.path.join(get_package_prefix('my_robot_gazebo'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    urdf_file_name = 'provatm.urdf'
    urdf_path = os.path.join(get_package_share_directory('my_robot_gazebo'), 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    infp.close()

    with open("/tmp/debug_log.txt", "w") as f:
        f.write(f"URDF Content: {robot_desc[:200]}\n")  # Stampa solo i primi 200 caratteri
    f.close()


    print(f"robot_description: {robot_desc[:200]}")  # Mostra solo i primi 200 caratteri per verifica

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
        # DeclareLaunchArgument(
        #     name='urdf_path',
        #     default_value=urdf_path,
        #     description='Path to URDF file'
        # ),

        # Avvia Gazebo
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),

        

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # Avvia joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': True}]
        ),
        

        # Avvia lo script Python per spawnare il robot
        Node(
            package="my_robot_gazebo",
            executable="spawn_robot",
            output="screen",
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),


        Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_control_config_path],
        output="screen"
        )

    ])
