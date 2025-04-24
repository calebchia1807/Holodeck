import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
import pickle

SHARED_MEMORY_BOUNDING_BOX = os.path.expanduser("~/bounding_box_shm")
SHARED_MEMORY_RGB = os.path.expanduser("~/rgb_shm")
SHM_SIZE_FRAME = 549 * 1158  * 3 * 4

def read_bounding_box_dict():
    with open(SHARED_MEMORY_BOUNDING_BOX, "rb") as shm:
        data = shm.read()
        return pickle.loads(data)

def read_rgb_frame():
    with open(SHARED_MEMORY_RGB, "rb") as shm:
        data = shm.read(SHM_SIZE_FRAME)
        return np.frombuffer(data, dtype=np.int32).reshape(549, 1158 , 3)

class BoundingBoxPublisher(Node):
    def __init__(self):
        super().__init__('unity_bounding_box_publisher')
        self.publisher = self.create_publisher(Image, 'bounding_box_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_bounding_box_image)

    def publish_bounding_box_image(self):
        rgb_frame = read_rgb_frame().astype(np.uint8)
        bounding_boxes = read_bounding_box_dict()

        for label, (x1, y1, x2, y2) in bounding_boxes.items():
            cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            object_name = label.split('|')[0]
            cv2.putText(rgb_frame, object_name, (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        ros_image = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_link"
        self.publisher.publish(ros_image)

        self.get_logger().info("Published image with bounding boxes.")

def main(args=None):
    rclpy.init(args=args)
    node = BoundingBoxPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
