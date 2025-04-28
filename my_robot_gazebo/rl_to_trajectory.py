#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RLJointController(Node):
    def __init__(self):
        super().__init__('rl_joint_controller')

        # 🔧 Nomi esatti dei giunti! Modifica se servono
        self.joint_names = ['shoulder_1_joint',
            'shoulder_2_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint']

        # 📩 Subscriber dal tuo RL agent
        self.create_subscription(Float64MultiArray, '/arm_action', self.arm_action_callback, 10)

        # 📤 Publisher al controller dei giunti
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.get_logger().info(f"NODO AVVIATO")

    def arm_action_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn('Dimensione comando diversa dal numero di giunti!')
            return

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(msg.data)
        point.time_from_start.sec = 3 # tempo per raggiungere la posizione

        traj_msg.points.append(point)

        self.publisher.publish(traj_msg)
        self.get_logger().info(f"Pubblicato comando: {point.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = RLJointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
