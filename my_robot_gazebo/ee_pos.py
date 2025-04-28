import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class EEListener(Node):
    def __init__(self):
        super().__init__('ee_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_ee_pose(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'world',  # frame di riferimento
                'wrist_3_link',  # link dell'end effector, i nomi da vere in tm5_900.urdf.xacro
                now
            )
            self.get_logger().info(f"EE position: x={trans.transform.translation.x}, y={trans.transform.translation.y}, z={trans.transform.translation.z}")
            return trans.transform
        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
            return None

def main():
    rclpy.init()
    node = EEListener()
    while rclpy.ok():
        rclpy.spin_once(node)
        node.get_ee_pose()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
