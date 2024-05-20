import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('Error_line')

        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/video_source/raw', self.image_callback, rclpy.qos.qos_profile_sensor_data)
        self.pub_error = self.create_publisher(Float32, 'error_line', 10) # Publica el color identificado en la imagen mediante un número
        self.pub_line_image = self.create_publisher(Image, '/img_line', 10) # Nodo para verificar la identificación de colores rojos en cámara
        
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.line_detection_callback)

        # Imagen
        self.cameraImg = np.zeros((480, 640, 3), dtype=np.uint8)
         # Mensaje de error  
        self.errorMsg = Float32()
        
        self.get_logger().info('Line detection Node Initialized')


    def image_callback(self, msg):
        try:
            self.cameraImg = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
        except Exception as e:
            self.get_logger().info(f'Failed to process image: {str(e)}')

    
    def line_detection_callback(self):
        self.errorMsg.data = self.calculoPendiente(self.cameraImg)
        self.pub_error.publish(self.errorMsg)

    def calculoPendiente(self,img):
        imgRecortadaRedim = self.resize_image(img)
        imgBinarizada = self.preprocess(imgRecortadaRedim)
        pendiente = self.pendiente_centroides(imgBinarizada)
        return pendiente

    ## Función que redimensiona y de la misma manera recorta la imágen
    def resize_image(self, img):
        # Se redimensiona la imágen
        ancho_r = img.shape[1] // 3  # Un tercio del ancho original
        alto_r = img.shape[0] // 3   # Un tercio del alto original
        img_redimensionada = cv2.resize(img, (ancho_r, alto_r))
        
        #Se recorta la imágen
        alto_original = img_redimensionada.shape[0]
        ancho_original = img_redimensionada.shape[1]
        inicio_y = int(2*alto_original // 3)  # la mitad del alto para el inicio del corte
        fin_y = alto_original  # El final del corte es el final de la imagen
        inicio_x = int(ancho_original // 3)
        fin_x = int(2 * ancho_original // 3)
        
        imgC = img_redimensionada[inicio_y:fin_y, inicio_x:fin_x]

        return imgC    

    # Función que hará el preprosesamiento
    def preprocess(self, imgC):
        # Se calcula el filtro medio
        filtro_median = cv2.medianBlur(imgC, 5)
        #Imagen a escala de grises
        img_g = cv2.cvtColor(filtro_median, cv2.COLOR_BGR2GRAY)
        # Se hace la binarización.
        _, imagen_binarizada = cv2.threshold(img_g, 85, 255, cv2.THRESH_BINARY)
        return imagen_binarizada
    
    # Función que calcula el error con los centroides
    def pendiente_centroides(self, img_bn):
        # Se hacen las operaciones morfológicas 
        SE_e = np.ones((15,15), np.uint8)
        morf_e = cv2.erode(img_bn, SE_e, iterations = 1)
        SE_d = np.ones((15,15), np.uint8)
        morf_d = cv2.dilate(morf_e, SE_d, iterations = 3)
        bordes = cv2.Canny(morf_d, 50, 150)
        centroide_ix = bordes.shape[1]/2
        centroide_iy = bordes.shape[0]/2

        # Encontrar contornos en la imagen
        contornos, _ = cv2.findContours(bordes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Asegúrate de que se han encontrado al menos dos contornos
        if len(contornos) >= 2:
            # Calcula los centroides de los dos contornos
            M1 = cv2.moments(contornos[0])
            cx1 = int(M1['m10'] / M1['m00'])
            cy1 = int(M1['m01'] / M1['m00'])

            M2 = cv2.moments(contornos[1])
            cx2 = int(M2['m10'] / M2['m00'])
            cy2 = int(M2['m01'] / M2['m00'])

            # Calcula el punto medio entre los dos centroides
            midpoint_x = (cx1 + cx2) // 2 
            midpoint_y = (cy1 + cy2) // 2
            pendiente = (centroide_iy - midpoint_y) / (centroide_ix - midpoint_x)
        else:
            pendiente = 0.0
        return pendiente
  

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
