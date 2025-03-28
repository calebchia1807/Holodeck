import socket
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class UnityController(Node):
    def __init__(self):
        super().__init__('unity_controller')
        # ROS2 Services
        self.move_ahead_srv = self.create_service(
            Trigger, '/simulator_ros/move_ahead', self.handle_move_ahead)
        self.move_left_srv = self.create_service(
            Trigger, '/simulator_ros/move_left', self.handle_move_left)
        self.move_back_srv = self.create_service(
            Trigger, '/simulator_ros/move_back', self.handle_move_back)
        self.move_right_srv = self.create_service(
            Trigger, '/simulator_ros/move_right', self.handle_move_right)
        
        # Socket connection
        self.socket_port = 8888
        self.get_logger().info(f"Unity controller ready, connecting to port {self.socket_port}")

    def send_socket_command(self, cmd):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(('localhost', self.socket_port))
                s.sendall(cmd.encode())
                response = s.recv(1024)
                return response == b"OK"
        except ConnectionRefusedError:
            self.get_logger().error("Unity connection refused - is connect_to_unity.py running?")
            return False
        except Exception as e:
            self.get_logger().error(f"Socket error: {str(e)}")
            return False

    def handle_move_ahead(self, request, response):
        response.success = self.send_socket_command('w')
        response.message = "Moved ahead" if response.success else "Movement failed"
        return response

    def handle_move_left(self, request, response):
        response.success = self.send_socket_command('a')
        response.message = "Moved left" if response.success else "Movement failed"
        return response

    def handle_move_back(self, request, response):
        response.success = self.send_socket_command('s')
        response.message = "Moved back" if response.success else "Movement failed"
        return response

    def handle_move_right(self, request, response):
        response.success = self.send_socket_command('d')
        response.message = "Moved right" if response.success else "Movement failed"
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = UnityController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()