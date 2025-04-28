#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(Point, '/target_point', 10)
        self.timer = self.create_timer(90.0, self.timer_callback)  # ogni 20 secondi cambia target
        self.get_logger().info('TargetPublisher avviato.')

    def timer_callback(self):
        target = Point()
        target.x = random.uniform(-0.8, 0.8)
        target.y = random.uniform(-0.3, -0.6)
        target.z = random.uniform(0.1, 0.8)
        self.publisher_.publish(target)
        self.get_logger().info(f'Nuovo target pubblicato: ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()