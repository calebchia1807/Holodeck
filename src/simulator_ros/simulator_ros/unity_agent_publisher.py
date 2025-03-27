import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tty, sys, termios

class UnityAgentPublisher(Node):
    def __init__(self):
        super().__init__('unity_agent_publisher')
        self.pub = self.create_publisher(String, '/keyboard_input', 10)
        self.get_logger().info("Press WASD to control THOR. Ctrl+C to exit.")
        
    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        while rclpy.ok():
            key = self.read_key()
            if key.lower() in ['w', 'a', 's', 'd']:
                msg = String()
                msg.data = key
                self.pub.publish(msg)
            elif key == '\x03':  # Ctrl+C
                break

def main(args=None):
    rclpy.init(args=args)
    node = UnityAgentPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()