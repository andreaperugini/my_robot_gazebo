#!/usr/bin/env python3

import rclpy
import os
import xacro
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory

class RobotSpawner(Node):
    def __init__(self):
        super().__init__("robot_spawner")

        self.cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Attendo il servizio /spawn_entity...")
        self.cli.wait_for_service()

        # Percorso del file URDF/Xacro
        #package_path = os.popen("ros2 pkg prefix my_robot_gazebo").read().strip()
        #urdf_file = os.path.join(package_path, "../src/my_robot_gazebo/urdf/braccio.urdf.xacro")

        # Converti Xacro in URDF
        #self.get_logger().info(f"Caricando file: {urdf_file}")
        #urdf_content = xacro.process_file(urdf_file).toxml()


        #Percorso del file URDF
        urdf_path = os.path.join(get_package_share_directory("my_robot_gazebo"),"urdf","provatm.urdf")
        

        # Posizione iniziale
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.5

        # Richiesta per spawnare il robot
        request = SpawnEntity.Request()
        request.name = "braccio_robotico"
        #request.xml = urdf_content
        request.xml = open(urdf_path, 'r').read()
        request.robot_namespace = ""
        request.initial_pose = initial_pose
        request.reference_frame = "world"

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Risultato spawn: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Errore nello spawn del robot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
