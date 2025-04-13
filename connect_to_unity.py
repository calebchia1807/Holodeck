import os
import json
import numpy as np
import socket
from argparse import ArgumentParser

import ai2thor
import compress_json
from ai2thor.controller import Controller
from ai2thor.hooks.procedural_asset_hook import ProceduralAssetHookRunner
from ai2holodeck.constants import HOLODECK_BASE_DATA_DIR, THOR_COMMIT_ID, OBJATHOR_ASSETS_DIR

SHARED_MEMORY_NAME_FRAME        = os.path.expanduser("~/frame_cam_shm")
SHARED_MEMORY_NAME_DEPTH        = os.path.expanduser("~/depth_cam_shm")
SHARED_MEMORY_NAME_SEGMENTATION = os.path.expanduser("~/segmentation_cam_shm")
SHM_SIZE_FRAME = 353 * 906 * 3 * 4
SHM_SIZE_DEPTH = 353 * 906 * 4
SHM_SIZE_SEGMENTATION = 353 * 906 * 3 * 4

def get_frames(controller):
    frame = controller.last_event.frame
    frame_bytes = frame.astype(np.int32).tobytes()
    with open(SHARED_MEMORY_NAME_FRAME, "r+b") as shm:
        shm.write(frame_bytes)

def get_depth_frames(controller):
    depth_frame = controller.last_event.depth_frame
    depth_bytes = depth_frame.astype(np.float32).tobytes()
    with open(SHARED_MEMORY_NAME_DEPTH, "r+b") as shm:
        shm.write(depth_bytes)

def get_segmentation_frames(controller):
    segmentation_frame = controller.last_event.instance_segmentation_frame
    segmentation_bytes = segmentation_frame.astype(np.int32).tobytes()
    with open(SHARED_MEMORY_NAME_SEGMENTATION, "r+b") as shm:
        shm.write(segmentation_bytes)

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
        gridSize=0.001,
        renderDepthImage=True,
        renderInstanceSegmentation=True,
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
    with open(SHARED_MEMORY_NAME_FRAME, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_FRAME)
    with open(SHARED_MEMORY_NAME_DEPTH, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_DEPTH)
    with open(SHARED_MEMORY_NAME_SEGMENTATION, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_SEGMENTATION)

    controller.step(action="CreateHouse", house=scene)
    get_frames(controller)
    get_depth_frames(controller)
    get_segmentation_frames(controller)
    print("Scene initialized.")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 8888))
    server_socket.listen()
    print("Waiting for commands...")
    try:
        while True:
            client_socket, addr = server_socket.accept()
            with client_socket:
                command = client_socket.recv(1024).decode()
                command_dict = json.loads(command)
                controller.step(**command_dict)
                get_frames(controller)
                get_depth_frames(controller)
                get_segmentation_frames(controller)
    except KeyboardInterrupt:
        server_socket.close()
        os.remove(SHARED_MEMORY_NAME_DEPTH)
        print("Shutting down...")

if __name__ == "__main__":
    main()
