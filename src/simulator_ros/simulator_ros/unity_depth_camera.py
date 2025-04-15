import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import cv2

SHARED_MEMORY_DEPTH = os.path.expanduser("~/depth_shm")
SHM_SIZE_FRAME = 353 * 906 * 4

def read_depth_frame():
    with open(SHARED_MEMORY_DEPTH, "rb") as shm:
        data = shm.read(SHM_SIZE_FRAME)
        return np.frombuffer(data, dtype=np.float32).reshape(353, 906)

def normalize_depth(depth_frame):
    if np.isnan(depth_frame).any():
        depth_frame = np.nan_to_num(depth_frame, nan=0.0)

    min_val = np.min(depth_frame)
    max_val = np.max(depth_frame)
    if max_val - min_val == 0:
        return np.zeros_like(depth_frame)

    normalized = (depth_frame - min_val) / (max_val - min_val)
    return normalized


class DepthImagePublisher(Node):
    def __init__(self):
        super().__init__('unity_depth_image_publisher')
        self.publisher = self.create_publisher(Image, 'depth_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_depth_image)

    def publish_depth_image(self):
        depth_frame = read_depth_frame()
        normalized_depth = normalize_depth(depth_frame)*10000
        scaled_depth = (normalized_depth * 255.0).clip(0, 255).astype(np.float32)

        ros_image = self.bridge.cv2_to_imgmsg(scaled_depth, encoding='32FC1')
        ros_image.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(ros_image)
        self.get_logger().info("Published depth image.")

def main(args=None):
    rclpy.init(args=args)
    node = DepthImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


'''
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
'''