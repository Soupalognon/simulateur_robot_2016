import rclpy
from rclpy.node import Node
from webots_ros2_msgs.msg import CameraRecognitionObjects
from std_msgs.msg import ColorRGBA
from robot_wall_a_msgs.msg import ColorRecognition, ColorRecognitionArray

class ColorSubscriber(Node):
    def __init__(self):
        super().__init__('color_listener')

        self.colors = ColorRecognitionArray()
        self.topic_prefix = 'camera_'

        for i in range(4):
            self.colors.colors.append(ColorRecognition())

            self.subscription = self.create_subscription(
                CameraRecognitionObjects,
                f'/{self.topic_prefix}{i+1}/recognitions/webots',
                self.listener_callback,
                10)
        
        self.publisher_ = self.create_publisher(ColorRecognitionArray, 'tiles_color', 10)

    def listener_callback(self, msg: CameraRecognitionObjects):
        color = None
        if not msg.objects:
            self.get_logger().info("Aucun objet détecté")
            return
        for obj in msg.objects:
            self.get_logger().info(f"{msg.header.frame_id} -> Object: {obj.id} / Color: {obj.colors}")
            if(obj.colors[0] == ColorRGBA(r=0.0,g=0.0,b=1.0, a=0.0)):
                color = ColorRecognition.BLUE
                break
            elif(obj.colors[0] == ColorRGBA(r=1.0,g=1.0,b=0.0, a=0.0)):
                color = ColorRecognition.YELLOW
                break

        if(color == None):
            self.get_logger().info("Color not recognized. Do not publish")
            return

        if(msg.header.frame_id == 'camera_1'):
            self.colors.colors[0].object_id = obj.id
            self.colors.colors[0].color = color
        elif(msg.header.frame_id == 'camera_2'):
            self.colors.colors[1].object_id = obj.id
            self.colors.colors[1].color = color
        elif(msg.header.frame_id == 'camera_3'):
            self.colors.colors[2].object_id = obj.id
            self.colors.colors[2].color = color
        elif(msg.header.frame_id == 'camera_4'):
            self.colors.colors[3].object_id = obj.id
            self.colors.colors[3].color = color

        self.publisher_.publish(self.colors)


def main():
    rclpy.init()
    node = ColorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
