#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import tf2_ros
import geometry_msgs.msg

class EEListener(Node):
    def __init__(self):
        super().__init__('ee_listener_node')
        
        # Publisher per le coordinate dell'end-effector
        self.publisher = self.create_publisher(PoseStamped, '/ee_pose', 10)
        
        # Creazione del buffer e del listener per il tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Esegui la lettura delle coordinate ogni 0.1 secondi
        self.timer = self.create_timer(0.1, self.publish_ee_pose)

    def publish_ee_pose(self):
        try:
            # Ottieni la trasformazione tra il frame di riferimento del robot e quello dell'end-effector
            trans = self.tf_buffer.lookup_transform('world', 'tool0', rclpy.time.Time())
            
            # Crea un messaggio PoseStamped con i dati della trasformazione
            pose_msg = PoseStamped()
            #pose_msg.header.stamp = rclpy.time.Time()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation
            
            # Pubblica il messaggio
            self.publisher.publish(pose_msg)
        except (tf2_ros.TransformException) as e:
            self.get_logger().info('Could not get transform: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    ee_listener = EEListener()
    rclpy.spin(ee_listener)
    ee_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
