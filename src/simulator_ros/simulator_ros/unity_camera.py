import os
import numpy as np
from PIL import Image

SHARED_MEMORY_NAME = os.path.expanduser("~/unity_cam_shm")
FRAME_SIZE = 300 * 300 * 4  # 300x300 float32 array

exponent_calculated = False
exponent = 0

def avg_exponent(arr):
    non_zero_vals = arr[arr > 0]
    if non_zero_vals.size == 0:
        return None
    
    max_val = np.max(non_zero_vals)
    max_exponent = int(np.floor(np.log10(max_val)))
    min_val = np.min(non_zero_vals)
    min_exponent = int(np.floor(np.log10(min_val)))
    avg_exponent = abs(max_exponent + min_exponent) / 2
    return avg_exponent

def depth_frame_correction(depth_frame):
    global exponent_calculated, exponent
    if exponent_calculated == False:
        exponent = avg_exponent(depth_frame)
        exponent_calculated = True
    upped_depth_frame = (depth_frame * float(10**exponent))
    return upped_depth_frame

def read_depth_frame():
    with open(SHARED_MEMORY_NAME, "rb") as shm:
        data = shm.read(FRAME_SIZE)
        depth_frame = np.frombuffer(data, dtype=np.float32).reshape(300, 300)
        return depth_frame

while True:
    depth_frame = read_depth_frame()
    print(depth_frame)
    upped_depth_frame = depth_frame_correction(depth_frame)
    print("Received Depth Frame:", upped_depth_frame)
    img = Image.fromarray(upped_depth_frame)
    img.show(title='Depth')


    

    # img = Image.fromarray(upped_depth_frame)
    # img.show(title='Depth Image')