import os
import numpy as np
from PIL import Image

SHARED_MEMORY_NAME = os.path.expanduser("~/unity_cam_shm")
SHM_SIZE = 353 * 906 * 4 

def read_depth_frame():
    with open(SHARED_MEMORY_NAME, "rb") as shm:
        data = shm.read(SHM_SIZE)
        depth_frame = np.frombuffer(data, dtype=np.float32).reshape(353, 906)
        return depth_frame

while True:
    depth_frame = read_depth_frame()
    print("Received Depth Frame:")
    print(depth_frame)
    img = Image.fromarray(depth_frame)
    img.show(title='Depth Image')