import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import ai2thor
from ai2thor.controller import Controller
from ai2thor.hooks.procedural_asset_hook import ProceduralAssetHookRunner
import compress_json
import os

from ai2holodeck.constants import (
    HOLODECK_BASE_DATA_DIR,
    THOR_COMMIT_ID,
    OBJATHOR_ASSETS_DIR,
)

class UnityAgentController(Node):
    def __init__(self):
        super().__init__('unity_agent_controller')
        self.subscription = self.create_subscription(
            String,
            'move_cmd',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Load scene
        scene_path = os.path.join(HOLODECK_BASE_DATA_DIR, "scenes/a_living_room/a_living_room.json")
        scene = compress_json.load(scene_path)

        # Init controller
        self.controller = Controller(
            commit_id=THOR_COMMIT_ID,
            start_unity=False,
            port=8200,
            scene="Procedural",
            gridSize=0.25,
            width=300,
            height=300,
            makeAgentsVisible=False,
            visibilityScheme="Distance",
            action_hook_runner=ProceduralAssetHookRunner(
                asset_directory=OBJATHOR_ASSETS_DIR,
                asset_symlink=True,
                verbose=True,
            ),
        )

        self.controller.step(action="CreateHouse", house=scene)
        self.get_logger().info("AI2-THOR controller ready.")

        self.key_action_map = {
            'w': 'MoveAhead',
            'a': 'MoveLeft',
            's': 'MoveBack',
            'd': 'MoveRight',
        }

    def listener_callback(self, msg: String):
        key = msg.data.lower()
        action = self.key_action_map.get(key)

        if action:
            event = self.controller.step(action=action)
            self.get_logger().info(f"Executed action: {action}")
        else:
            self.get_logger().warn(f"Unmapped key received: {key}")

def main(args=None):
    rclpy.init(args=args)
    node = UnityAgentController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
