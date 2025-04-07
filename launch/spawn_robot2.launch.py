from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix



def generate_launch_description():

    package_name = 'my_robot_gazebo'
    urdf_file_name = 'provatm.urdf'
    package_path = get_package_share_directory(package_name)

     #URDF
    urdf_path = os.path.join(package_path, 'urdf', urdf_file_name)
    
    ros2_control_config_path = os.path.join(package_path, 'config', 'ros2_control.yaml')
    

    pkg_gazebo_ros = os.popen("ros2 pkg prefix gazebo_ros").read().strip()
    gazebo_launch = os.path.join(pkg_gazebo_ros, "share", "gazebo_ros", "launch", "gazebo.launch.py")

    pkg_share_path = os.path.join(get_package_prefix('my_robot_gazebo'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    
    

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    infp.close()

   
    # Controllo se il file esiste
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"File URDF non trovato: {urdf_path}")
    # Lettura del file URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()



    #NODI(non tutti sono stati definiti qui, alcuni direttamente sotto)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )


    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )





    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf_path',
            default_value=urdf_path,
            description='Path to URDF file'
        ),

        # Avvia Gazebo
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),


        robot_state_publisher_node,

        # Avvia lo script Python per spawnare il robot
        Node(
            package="my_robot_gazebo",
            executable="spawn_robot",
            output="screen",
            arguments=['-topic', 'robot_description', '-entity', 'robot']
        ),

        load_joint_state_broadcaster,
        load_joint_trajectory_controller
       

    ])
