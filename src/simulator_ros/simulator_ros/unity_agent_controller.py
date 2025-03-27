import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # For keyboard input (can be replaced with custom msg)
import ai2thor
from ai2thor.controller import Controller

class UnityAgentController(Node):
    def __init__(self):
        super().__init__('unity_agent_controller')
        
        # Initialize AI2-THOR controller (similar to your connect_to_unity.py)
        self.controller = Controller(
            scene="Procedural",
            gridSize=0.25,
            width=300,
            height=300,
        )
        self.controller.reset('FloorPlan1')  # Default scene
        
        # Subscribe to keyboard input (e.g., from a teleop node)
        self.key_sub = self.create_subscription(
            String, '/keyboard_input', self.key_callback, 10
        )
        self.get_logger().info("Keyboard-THOR controller ready. Waiting for inputs...")

    def key_callback(self, msg):
        key = msg.data.lower()
        action = None
        
        # Map keys to THOR actions
        if key == 'w':
            action = 'MoveAhead'
        elif key == 's':
            action = 'MoveBack'
        elif key == 'a':
            action = 'MoveLeft'
        elif key == 'd':
            action = 'MoveRight'
        else:
            self.get_logger().warn(f"Unsupported key: {key}")
            return
        
        # Execute action in THOR
        event = self.controller.step(dict(action=action))
        self.get_logger().info(f"Executed: {action}")

def main(args=None):
    rclpy.init(args=args)
    node = UnityAgentController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()