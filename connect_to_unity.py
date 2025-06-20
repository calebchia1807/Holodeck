import os
import json
import socket
import pickle
import numpy as np
from argparse import ArgumentParser

import ai2thor
import compress_json
from ai2thor.controller import Controller
from ai2thor.hooks.procedural_asset_hook import ProceduralAssetHookRunner
from ai2holodeck.constants import HOLODECK_BASE_DATA_DIR, THOR_COMMIT_ID, OBJATHOR_ASSETS_DIR

# shared memory to send respective the frame data
SHARED_MEMORY_FRAME_DIMENSIONS = os.path.expanduser("~/frame_dimensions_shm")
SHARED_MEMORY_RGB              = os.path.expanduser("~/rgb_shm")
SHARED_MEMORY_BGR              = os.path.expanduser("~/bgr_shm")
SHARED_MEMORY_DEPTH            = os.path.expanduser("~/depth_shm")
SHARED_MEMORY_SEGMENTATION     = os.path.expanduser("~/segmentation_shm")
SHARED_MEMORY_BOUNDING_BOX     = os.path.expanduser("~/bounding_box_shm")

def get_rgb_frames(controller):
    rgb_frame = controller.last_event.frame
    rgb_bytes = rgb_frame.astype(np.int32).tobytes()
    with open(SHARED_MEMORY_RGB, "r+b") as shm:
        shm.write(rgb_bytes)

def get_bgr_frames(controller):
    bgr_frame = controller.last_event.cv2img
    bgr_bytes = bgr_frame.astype(np.int32).tobytes()
    with open(SHARED_MEMORY_BGR, "r+b") as shm:
        shm.write(bgr_bytes)

def get_depth_frames(controller):
    depth_frame = controller.last_event.depth_frame
    depth_bytes = depth_frame.astype(np.float32).tobytes()
    with open(SHARED_MEMORY_DEPTH, "r+b") as shm:
        shm.write(depth_bytes)

def get_segmentation_frames(controller):
    segmentation_frame = controller.last_event.instance_segmentation_frame
    segmentation_bytes = segmentation_frame.astype(np.int32).tobytes()
    with open(SHARED_MEMORY_SEGMENTATION, "r+b") as shm:
        shm.write(segmentation_bytes)

def get_bounding_box_frames(controller):
    get_rgb_frames(controller)
    bounding_box_frame = controller.last_event.instance_detections2D
    bounding_box_dict = dict(bounding_box_frame)
    bounding_box_bytes = pickle.dumps(bounding_box_dict)
    with open(SHARED_MEMORY_BOUNDING_BOX, "r+b") as shm:
        shm.write(bounding_box_bytes)

# holodeck code
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

    # the frame size always changes, this dynamically updates the frame
    height, width, channel = controller.last_event.frame.shape
    SHM_SIZE_FRAME = height * width * channel* 4  # for rgb, bgr & segmentation
    SHM_SIZE_DEPTH = height * width * 4
    SHM_SIZE_FRAME_DIMENSIONS = 3 * 4
    with open(SHARED_MEMORY_FRAME_DIMENSIONS, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_FRAME_DIMENSIONS)
    frame_dimensions_array = np.array([height, width, channel], dtype=np.int32)
    with open(SHARED_MEMORY_FRAME_DIMENSIONS, "r+b") as shm:
        shm.write(frame_dimensions_array.tobytes())
    
    # shared memory & control within unity
    with open(SHARED_MEMORY_RGB, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_FRAME)
    with open(SHARED_MEMORY_BGR, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_FRAME)
    with open(SHARED_MEMORY_DEPTH, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_DEPTH)
    with open(SHARED_MEMORY_SEGMENTATION, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_FRAME)
    with open(SHARED_MEMORY_BOUNDING_BOX, "wb") as shm:
        shm.write(b"\x00" * SHM_SIZE_FRAME)

    controller.step(action="CreateHouse", house=scene)
    get_rgb_frames(controller)
    get_bgr_frames(controller)
    get_depth_frames(controller)
    get_segmentation_frames(controller)
    get_bounding_box_frames(controller)
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
                get_rgb_frames(controller)
                get_bgr_frames(controller)
                get_depth_frames(controller)
                get_segmentation_frames(controller)
                get_bounding_box_frames(controller)
    except KeyboardInterrupt:
        server_socket.close()
        os.remove(SHARED_MEMORY_DEPTH)
        print("Shutting down...")

if __name__ == "__main__":
    main()
