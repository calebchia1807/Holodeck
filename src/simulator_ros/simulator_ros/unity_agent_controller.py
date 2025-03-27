import os
import compress_json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from argparse import ArgumentParser

import ai2thor
from ai2thor.controller import Controller
from ai2thor.hooks.procedural_asset_hook import ProceduralAssetHookRunner
from ai2holodeck.constants import HOLODECK_BASE_DATA_DIR, THOR_COMMIT_ID, OBJATHOR_ASSETS_DIR


class UnityAgentController(Node):
    def __init__(self, scene_path, asset_dir):
        super().__init__('unity_agent_controller')

        # Load the scene JSON file
        scene = compress_json.load(scene_path)

        # Initialize AI2-THOR controller
        self.controller = Controller(
            commit_id=THOR_COMMIT_ID,
            start_unity=False,
            port=8200,
            scene="Procedural",
            gridSize=0.25,
            width=300,
            height=300,
            server_class=ai2thor.wsgi_server.WsgiServer,
            makeAgentsVisible=False,
            visibilityScheme="Distance",
            action_hook_runner=ProceduralAssetHookRunner(
                asset_directory=asset_dir,
                asset_symlink=True,
                verbose=True,
            ),
        )

        # Create the house in Unity
        self.controller.step(action="CreateHouse", house=scene)
        self.get_logger().info(f"Controller initialized and house created from: {scene_path}")

        # Mapping keyboard keys to movement actions
        self.key_map = {
            'w': 'MoveAhead',
            'a': 'MoveLeft',
            's': 'MoveBack',
            'd': 'MoveRight',
        }

        # ROS 2 Subscriber for keyboard control
        self.subscription = self.create_subscription(
            String,
            'move_cmd',
            self.handle_keypress,
            10
        )

    def handle_keypress(self, msg: String):
        key = msg.data.lower()
        action = self.key_map.get(key)
        if action:
            self.controller.step(action=action)
            self.get_logger().info(f"Executed action: {action}")
        else:
            self.get_logger().warn(f"Invalid key received: '{key}'")


def main(args=None):
    # Parse command-line arguments
    parser = ArgumentParser()
    parser.add_argument(
        "--scene",
        help="Path to the scene JSON file",
        default=os.path.join(HOLODECK_BASE_DATA_DIR, "scenes/a_living_room/a_living_room.json")
    )
    parser.add_argument(
        "--asset_dir",
        help="Path to the asset directory",
        default=OBJATHOR_ASSETS_DIR
    )
    cli_args = parser.parse_args()

    # Initialize ROS node with provided arguments
    rclpy.init(args=args)
    node = UnityAgentController(scene_path=cli_args.scene, asset_dir=cli_args.asset_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()