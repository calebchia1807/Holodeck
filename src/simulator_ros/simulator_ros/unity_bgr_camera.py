import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import cv2

SHARED_MEMORY_FRAME_DIMENSIONS = os.path.expanduser("~/frame_dimensions_shm")
SHARED_MEMORY_BGR = os.path.expanduser("~/bgr_shm")

def read_frame_dimensions():
    with open(SHARED_MEMORY_FRAME_DIMENSIONS, "rb") as shm:
        data = shm.read(3 * 4)
        frame_dimensions = np.frombuffer(data, dtype=np.int32)
        return frame_dimensions[0], frame_dimensions[1], frame_dimensions[2]
    
def read_bgr_frame():
    height, width, channel = read_frame_dimensions()
    shm_size_frame = height * width * channel * 4
    with open(SHARED_MEMORY_BGR, "rb") as shm:
        data = shm.read(shm_size_frame)
        return np.frombuffer(data, dtype=np.int32).reshape(height, width, channel)
    
class BGRImagePublisher(Node):
    def __init__(self):
        super().__init__('unity_bgr_image_publisher')
        self.publisher = self.create_publisher(Image, 'bgr_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_bgr_image)

    def publish_bgr_image(self):
        bgr_frame = read_bgr_frame().astype(np.uint8)

        ros_image = self.bridge.cv2_to_imgmsg(bgr_frame, encoding='rgb8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_link"

        self.publisher.publish(ros_image)
        self.get_logger().info("Published BGR image.")
        
def main(args=None):
    rclpy.init(args=args)
    node = BGRImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()