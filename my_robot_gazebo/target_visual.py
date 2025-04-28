#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SpawnEntity, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import os
import subprocess

TARGET_TOPIC = '/target_point'

class TargetSpawner(Node):
    def __init__(self):
        super().__init__('target_spawner')

        # Client per spawn_entity
        self.cli_spawn = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn_entity service not available, waiting...')

        # Client per SetModelState
        self.cli_set_state = self.create_client(SetModelState, '/gazebo/set_model_state')
        while not self.cli_set_state.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('/gazebo/set_model_state service not available, waiting...')

        self.get_logger().info('/spawn_entity and /gazebo/set_model_state services available!')

        # Subscriber al target
        self.subscription = self.create_subscription(
            Point,
            TARGET_TOPIC,
            self.target_callback,
            10)

        self.target_pose = Pose()
        self.spawned = False

    def target_callback(self, msg):
        self.get_logger().info(f"Received new target at ({msg.x}, {msg.y}, {msg.z})")

        if not self.spawned:
            self.spawn_target()
            self.spawned = True
        else:
            # Set the target position in ModelState
            model_state = ModelState()
            model_state.model_name = 'target_sphere'  # Assicurati che il nome del modello corrisponda
            model_state.pose.position.x = msg.x
            model_state.pose.position.y = msg.y
            model_state.pose.position.z = msg.z
            model_state.pose.orientation.w = 1.0  # Puoi mantenere l'orientamento come 1.0 per evitare modifiche

            # Setta la posizione del modello
            self.set_model_state(model_state)

    def spawn_target(self):
        request = SpawnEntity.Request()

        # Assicurati che il nome del modello sia corretto
        request.name = 'target_sphere'
        request.xml = self.load_xacro()
        request.robot_namespace = ''
        request.initial_pose = self.target_pose
        request.reference_frame = 'world'

        self.get_logger().info('Sending spawn request...')
        future = self.cli_spawn.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Sphere successfully spawned in Gazebo with Xacro!')
            else:
                self.get_logger().error(f"Failed to spawn sphere: {future.result().status_message}")
        else:
            self.get_logger().error('Service call failed :(')

    def set_model_state(self, model_state: ModelState):
        # Controlla se il servizio è disponibile
        self.get_logger().info("Checking if /gazebo/set_model_state service is available...")
        
        if not self.cli_set_state.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/gazebo/set_model_state service still not available after waiting')
            return

        # Crea la richiesta per SetModelState
        request = SetModelState.Request()
        request.model_state = model_state

        # Invia la richiesta per aggiornare la posizione
        future = self.cli_set_state.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Successfully updated target position to ({model_state.pose.position.x}, {model_state.pose.position.y}, {model_state.pose.position.z})')
            else:
                self.get_logger().error(f"Failed to update target position: {future.result().status_message}")
        else:
            self.get_logger().error('Service call failed :(')

    def load_xacro(self):
        # Path assoluto al tuo file xacro
        xacro_path = os.path.join(
            os.getenv('HOME'),  # oppure usa pathlib se vuoi /home/gabriel/ros2_ws/src/my_robot_gazebo/urdf/sfera_target.xacro
            'gabriel','ros2_ws', 'src', 'my_robot_gazebo', 'urdf', 'sfera_target.xacro'
        )

        # Espande il file xacro in urdf usando il comando di sistema
        try:
            output = subprocess.check_output(['xacro', xacro_path])
            return output.decode('utf-8')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to process xacro: {e}")
            return ""

def main(args=None):
    rclpy.init(args=args)
    node = TargetSpawner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
