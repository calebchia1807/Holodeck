import os
import socket
from argparse import ArgumentParser

import ai2thor
import compress_json
from ai2thor.controller import Controller
from ai2thor.hooks.procedural_asset_hook import ProceduralAssetHookRunner
from ai2holodeck.constants import HOLODECK_BASE_DATA_DIR, THOR_COMMIT_ID, OBJATHOR_ASSETS_DIR

def main():
    parser = ArgumentParser()
    parser.add_argument(
        "--scene",
        help="the directory of the scene to be generated",
        default=os.path.join(
            HOLODECK_BASE_DATA_DIR, "/scenes/a_living_room/a_living_room.json"
        ),
    )   
    parser.add_argument(
        "--asset_dir",
        help="the directory of the assets to be used",
        default=OBJATHOR_ASSETS_DIR,
    )
    args = parser.parse_args()

    scene = compress_json.load(args.scene)

    controller = Controller(
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
            asset_directory=args.asset_dir,
            asset_symlink=True,
            verbose=True,
        ),
    )

    controller.step(action="CreateHouse", house=scene)
    print(f"Scene initialized.")
    print(f"Run python3 unity_agent_controller.py to control motion")
    
    # Set up socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # This allows address reuse
    server_socket.bind(('localhost', 8888))
    server_socket.listen()
    print("Waiting for movement commands... (Ctrl+C to exit)")

    try:
        while True:
            conn, addr = server_socket.accept()
            with conn:
                data = conn.recv(1024).decode().strip()
                
                if data == 'w':
                    res = controller.step("MoveAhead", moveMagnitude=0.25)
                elif data == 'a':
                    res = controller.step("MoveLeft", moveMagnitude=0.25)
                elif data == 's':
                    res = controller.step("MoveBack", moveMagnitude=0.25)
                elif data == 'd':
                    res = controller.step("MoveRight", moveMagnitude=0.25)
                
                conn.sendall(b"OK")
    except KeyboardInterrupt:
        print("\nShutting down server...")

if __name__ == "__main__":
    main()