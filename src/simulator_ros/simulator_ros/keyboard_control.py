import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(String, 'move_cmd', 10)
        self.get_logger().info("Use W/A/S/D to move. Press Ctrl+C to exit.")
        self.old_settings = termios.tcgetattr(sys.stdin)

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.read_key().lower()
                if key in ['w', 'a', 's', 'd']:
                    msg = String()
                    msg.data = key
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Key pressed: {key}")
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down keyboard controller.")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()