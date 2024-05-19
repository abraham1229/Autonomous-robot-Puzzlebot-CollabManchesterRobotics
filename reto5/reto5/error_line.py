import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('Error_line')

        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/video_source/raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.pub_error = self.create_publisher(Int32, 'error_line', 10) # Publica el color identificado en la imagen mediante un número
        self.pub_line_image = self.create_publisher(Image, '/img_line', 10) # Nodo para verificar la identificación de colores rojos en cámara
        
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.line_detection_callback)

        # Imagen
        self.cameraImg = np.zeros((480, 640, 3), dtype=np.uint8)
        # Mensaje de error  
        self.errorMsg = Int32()

        self.valid_img = False
        self.get_logger().info('Line detection Node Initialized')


    def image_callback(self, msg):
        try:
            self.cameraImg = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except Exception as e:
            self.get_logger().info(f'Failed to process image: {str(e)}')

    
    def line_detection_callback(self):
        self.errorMsg.data = 0
        self.pub_error.publish(self.errorMsg)

  

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
