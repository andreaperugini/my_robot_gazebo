from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix



def generate_launch_description():


    pkg_gazebo_ros = os.popen("ros2 pkg prefix gazebo_ros").read().strip()
    gazebo_launch = os.path.join(pkg_gazebo_ros, "share", "gazebo_ros", "launch", "gazebo.launch.py")


    #NECESSARIO ALTRIMENTI NON CARICA LE MESHES
    pkg_share_path = os.path.join(get_package_prefix('my_robot_gazebo'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path


    # Dichiarazione degli argomenti
    stl_mesh_arg = DeclareLaunchArgument(
        'stl_mesh',
        default_value='false',
        description='If true, use STL mesh for visuals'
    )

    transmission_hw_interface_arg = DeclareLaunchArgument(
        'transmission_hw_interface',
        default_value='hardware_interface/PositionJointInterface',
        description='Transmission hardware interface type'
    )

    # Path al file xacro
    package_dir = get_package_share_directory('my_robot_gazebo')
    xacro_file = os.path.join(package_dir, 'urdf', 'tm5_900_robot.urdf.xacro')

    # Comando xacro per generare il robot_description
    robot_description_content = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' stl_mesh:=', LaunchConfiguration('stl_mesh'),
            ' transmission_hw_interface:=', LaunchConfiguration('transmission_hw_interface')
        ]),
        value_type=str
    )

    robot_description_param = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param]
        )
    
    spawn_entity_mio = Node(
        package='my_robot_gazebo',
        executable='spawn_entity',
        output="screen",
        arguments=['-topic /robot_description']

    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output="screen",
        arguments=['-entity', 'my_robot','-topic','/robot_description','-x','0','-y','0','-z','1']

    )


    
       
    return LaunchDescription([


          # Avvia Gazebo
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),


        stl_mesh_arg,
        transmission_hw_interface_arg,

        robot_state_publisher_node,
        spawn_entity
        
        
        
       
        
    ])
