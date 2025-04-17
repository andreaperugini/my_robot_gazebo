import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import random
import numpy as np

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        # Publisher sul topic del tuo controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(5.0, self.send_trajectory)

    def send_trajectory(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'shoulder_1_joint',
            'shoulder_2_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        angoli = (3.14*np.random.random(6) - 3.14/2).round(2)
        point.positions = angoli.tolist()
        #point.positions = [0.0, -0.5, 1.0, 0.5, 0.0, 0.0]  # in radianti
        

      #  point.positions = [[random.uniform(-1.5, 1.5) for _ in traj_msg.joint_names]]
        point.time_from_start.sec = 3  # 3 secondi per raggiungere questa posizione

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info('Traiettoria pubblicata, gli angoli dei joints saranno:' + str(angoli))


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
