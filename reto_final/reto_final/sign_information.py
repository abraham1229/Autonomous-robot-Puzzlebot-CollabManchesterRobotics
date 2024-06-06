from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from msgs_clase.msg import Signal   # type: ignore

from yolov8_msgs.msg import InferenceResult # type: ignore
from yolov8_msgs.msg import Yolov8Inference # type: ignore

bridge = CvBridge()

class Sign_information(Node):

    def __init__(self):
        super().__init__('Sign_information')

        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10) 

        self.img_pub = self.create_publisher(Image, "/sign_information", 10)
        
        self.timer_period = 0.15
        self.timer = self.create_timer(self.timer_period, self.timer_callback_signs)

        self.img = np.ones((480, 640, 3), dtype=np.uint8)
        self.get_logger().info('Sign information node initialized')

    def camera_callback(self, data):
        if data is not None:
            self.img = bridge.imgmsg_to_cv2(data, "bgr8")
            self.img = cv2.rotate(self.img, cv2.ROTATE_180)
            self.img = cv2.resize(self.img, (340, 340))


    def timer_callback_signs(self):
        
        img_msg = bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
        self.img_pub.publish(img_msg)
       

def main(args=None):
    rclpy.init(args=args)
    sign_information = Sign_information()
    rclpy.spin(sign_information)
    sign_information.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
