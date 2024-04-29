import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub_color = self.create_publisher(Int32, 'color_detection', 10) # Publica el color identificado en la imagen mediante un número
        self.pub_red = self.create_publisher(Image, '/img_processing/red', 10) # Nodo para verificar la identificación de colores rojos en cámara
        self.pub_green = self.create_publisher(Image, '/img_processing/green', 10) # Nodo para verificar la identificación de colores verdes en cámara
        self.pub_yellow = self.create_publisher(Image, '/img_processing/yellow', 10) # Nodo para verificar la identificación de colores amarillos en cámara
 
        # Máscara para identificar colores rojos 
        self.lower_red = np.array([0, 87, 111])
        self.upper_red = np.array([10, 255, 255])

        # Máscara para identificar colores verdes 
        self.lower_green = np.array([50, 40, 40])
        self.upper_green = np.array([80, 255, 255])

        ## Máscara para identificar colores amarillos
        self.lower_yellow = np.array([25, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

        self.valid_img = False
        self.get_logger().info('Color Detection Node Initialized')

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
            self.detect_color(img)
        except Exception as e:
            self.get_logger().info(f'Failed to process image: {str(e)}')

    def detect_color(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Definir las dimensiones del elemento estructurante
        SE = np.ones((20, 20), np.uint8)

        #### Detección de ROJO ####
        mask_red = cv2.inRange(hsv_img, self.lower_red, self.upper_red)
        detected_red = cv2.bitwise_and(img, img, mask=mask_red)
        blurred_red = cv2.medianBlur(detected_red, 9)
        eroded_red = cv2.erode(blurred_red, SE, iterations=1)
        dilated_red = cv2.dilate(eroded_red, SE, iterations=1)
        red_edges = cv2.Canny(dilated_red, 75, 250)
        self.pub_red.publish(self.bridge.cv2_to_imgmsg(red_edges))

        #Detección de VERDE 
        mask_green = cv2.inRange(hsv_img, self.lower_green, self.upper_green)
        detected_green = cv2.bitwise_and(img, img, mask=mask_green)
        blurred_green = cv2.medianBlur(detected_green, 9)
        eroded_green = cv2.erode(blurred_green, SE, iterations=1)
        dilated_green = cv2.dilate(eroded_green, SE, iterations=1)
        green_edges = cv2.Canny(dilated_green, 75, 250)
        self.pub_green.publish(self.bridge.cv2_to_imgmsg(green_edges))

        # Detección de AMARILLOS 
        mask_yellow = cv2.inRange(hsv_img, self.lower_yellow, self.upper_yellow)
        detected_yellow = cv2.bitwise_and(img, img, mask=mask_yellow)
        blurred_yellow = cv2.medianBlur(detected_yellow, 9)
        eroded_yellow = cv2.erode(blurred_yellow, SE, iterations=1)
        dilated_yellow = cv2.dilate(eroded_yellow, SE, iterations=1)
        yellow_edges = cv2.Canny(dilated_yellow, 75, 250)
        self.pub_yellow.publish(self.bridge.cv2_to_imgmsg(yellow_edges))

        # Contar los elementos detectados 
        red_count = np.count_nonzero(red_edges)
        green_count = np.count_nonzero(green_edges)
        yellow_count = np.count_nonzero(yellow_edges)

        # Determinar el elemento con mayor incidencia 
        max_count = max(red_count, green_count, yellow_count)
        
        # Publicar en el nodo los valores en función del color #
        color_msg = Int32()
        if max_count == 0:
            color_msg.data = 0
        elif max_count == red_count:
            color_msg.data = 1
        elif max_count == green_count:
            color_msg.data = 2
        elif max_count == yellow_count:
            color_msg.data = 3
        
        self.pub_color.publish(color_msg)
  

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
