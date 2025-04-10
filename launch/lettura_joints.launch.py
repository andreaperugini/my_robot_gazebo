# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from launch.substitutions import LaunchConfiguration
import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)



def generate_launch_description():

      #NECESSARIO ALTRIMENTI NON CARICA LE MESHES
    pkg_share_path = os.path.join(get_package_prefix('my_robot_gazebo'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    package_dir = get_package_share_directory('my_robot_gazebo')

    xacro_file = os.path.join(package_dir,
                              'urdf',
                              'tm5_900_robot.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}
    #remove_comments(params)
    
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )
    #remove_comments(robot_description_content)

    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        #parameters=[{"robot_description": robot_description_content}]   )    
         parameters=[params]
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'braccio_robotico'],
                        output='screen')
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output="screen",
        arguments=['-entity', 'my_robot','-topic','robot_description',
                   '-x','0',
                   '-y','0',
                   '-z','0.5']
    )

    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # load_tricycle_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'tricycle_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_tricycle_controller],
        #     )
        # ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity
        
    ])
