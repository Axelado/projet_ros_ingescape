import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped


class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(TransformStamped, 'base_link_to_map_transform', 10)

        self.timer = self.create_timer(0.8, self.get_transform)  # Appelle la fonction toutes les 0.8 secondes

    def get_transform(self):
        try:
            # Récupère la transformation de 'base_link' par rapport à 'map'
            transform = self.buffer.lookup_transform('map', 'pizibot_base_link', rclpy.time.Time())
            
            # Publie la transformation sur un topic
            self.publisher.publish(transform)
            
            self.get_logger().info(
                f"Translation: x={transform.transform.translation.x}, "
                f"y={transform.transform.translation.y}, "
                f"z={transform.transform.translation.z}\n"
                f"Rotation (quaternion): x={transform.transform.rotation.x}, "
                f"y={transform.transform.rotation.y}, "
                f"z={transform.transform.rotation.z}, "
                f"w={transform.transform.rotation.w}"
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
