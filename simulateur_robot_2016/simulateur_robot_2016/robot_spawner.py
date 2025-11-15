import rclpy
from webots_ros2_msgs.srv import SpawnNodeFromString
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('spawn_robot_client')
    client = node.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service /Ros2Supervisor/spawn_node_from_string not available')
        return
    
    robot_name = sys.argv[1] if len(sys.argv) > 1 else 'Assemblage_Carcasse'
    robot_color_side = sys.argv[2] if len(sys.argv) > 2 else 'BLUE'
    # translation = sys.argv[2] if len(sys.argv) > 2 else '0 0 0'
    # rotation = sys.argv[3] if len(sys.argv) > 3 else '0 0 1 0'

    if(robot_color_side == "BLUE"):
        translation = '2.7 1.775 0.015'
        rotation = '0 0 1 -1.57'
    elif(robot_color_side == "YELLOW"):
        translation = '0.3 1.775 0.015'
        rotation = '0 0 1 -1.57'
    else:
        node.get_logger().error('Unknown color (BLUE or YELLOW)')
        return

    message = (
        f'{robot_name} '
        '{ '
        f'name "{robot_name}" '
        f'translation {translation} '
        f'rotation {rotation} '
        'controller "<extern>" '
        'controllerArgs [""] '
        '}'
    )
    node.get_logger().info("Spawner value = " + message)

    req = SpawnNodeFromString.Request()
    req.data = message
    # print(message)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Robot spawned: {future.result().success}")
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
