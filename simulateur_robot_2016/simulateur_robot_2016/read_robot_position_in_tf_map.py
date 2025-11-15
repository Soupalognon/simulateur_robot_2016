import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf_transformations


class TFReader(Node):
    def __init__(self):
        super().__init__('tf_reader')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Vérifie le TF 10 fois/s
        self.timer = self.create_timer(0.1, self.read_tf)

    def read_tf(self):
        try:
            # target_frame = frame dans lequel on veut la pose
            # source_frame = frame du robot
            transform = self.tf_buffer.lookup_transform(
                'map',        # target
                'base_link',   # source
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            quaternion = (q.x, q.y, q.z, q.w)

            # Conversion quaternion -> euler (roll, pitch, yaw)
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

            self.get_logger().info(
                f"Robot position in map: x={x:.3f}, y={y:.3f}, theta={yaw:.3f} rad"
            )

        except Exception as e:
            self.get_logger().warn(f"TF not available yet: {e}")


def main():
    rclpy.init()
    node = TFReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
