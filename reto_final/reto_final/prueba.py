import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
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
        self.pub_frenado = self.create_publisher(Int32, 'frenado', 10) # Nodo para verificar la identificación de colores rojos en cámara
        
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.line_detection_callback)
       
        self.bandera = Int32()  
        self.contador = 0
        self.cuadricula_ant = 0
        
        # Imagen
        self.cameraImg = np.ones((480, 640, 3), dtype=np.uint8)
        self.imgLecture = False
         # Mensaje de error  
        self.errorMsg = Float32()


        # Imágenes de procesos 
        self.imagenCortada    = np.ones((480, 640, 3), dtype=np.uint8)
        self.imagenBN  = np.ones((480, 640, 3), dtype=np.uint8)
        
        self.imagenProcesada = np.ones((480, 640, 1), dtype=np.uint8)
        

        self.xlast = 240
        self.ylast = 90
        self.get_logger().info('Line detection Node Initialized')


    def image_callback(self, msg):
        if msg is not None:
            self.cameraImg = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.imgLecture = True
        else:
            self.imgLecture = False

        # try:
        #     self.cameraImg = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #     self.imgLecture = True
            
        # except Exception as e:
        #     self.imgLecture = False
        #     self.get_logger().info(f'Failed to process image: {str(e)}')

    
    def line_detection_callback(self):
        if self.imgLecture:
            self.errorMsg.data = self.calculoError(self.cameraImg)
            self.pub_error.publish(self.errorMsg)
            self.pub_frenado.publish(self.bandera)
            
            #self.pub_line_image.publish(self.bridge.cv2_to_imgmsg(self.imagenProcesada))
            self.pub_line_image.publish(self.bridge.cv2_to_imgmsg(self.imagenProcesada,encoding="bgr8"))
            #self.get_logger().info(f'{self.errorMsg.data})')
        else:
            self.get_logger().info(f'Failed to process image')


    def calculoError(self,img):
        self.resize_image(img)
        self.preprocess(self.imagenCortada)
        error = self.pendiente_centroides(self.imagenBN,self.imagenCortada)
        return error

    ## Función que redimensiona y de la misma manera recorta la imágen
    def resize_image(self, img):
        # Se redimensiona la imágen
        ancho_r = img.shape[1] // 2  # Un tercio del ancho original
        alto_r = img.shape[0] // 2  # Un tercio del alto original
        img_redimensionada = cv2.resize(img, (ancho_r, alto_r))
        
        #Se recorta la imágen
        alto_original = img_redimensionada.shape[0]
        ancho_original = img_redimensionada.shape[1]
        inicio_y = 0  # la mitad del alto para el inicio del corte
        fin_y = alto_original-int(alto_original/2) # El final del corte es el final de la imagen
        inicio_x = int(ancho_original // 8)
        fin_x = int(7 * ancho_original // 8)
        
        imgC = img_redimensionada[inicio_y:fin_y, inicio_x:fin_x]

        self.imagenCortada = imgC   

    # Función que hará el preprosesamiento
    def preprocess(self, imgC):
        # Se calcula el filtro medio
        filtro_median = cv2.medianBlur(imgC, 3)
        #Imagen a escala de grises
        img_g = cv2.cvtColor(filtro_median, cv2.COLOR_BGR2GRAY)
        
        
        # Se hace la binarización normal
        # _, imagen_binarizada = cv2.threshold(img_g, 85, 255, cv2.THRESH_BINARY)
        # self.imagenProcesada = imagen_binarizada

        # Binarización de tipo Otsu
        # Apply Otsu's thresholding
        _, imagen_binarizada = cv2.threshold(img_g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        imagen_binarizada = cv2.bitwise_not(imagen_binarizada)
        self.imagenProcesada = imagen_binarizada
        self.imagenBN = imagen_binarizada
        
    # Función que calcula el error con los centroides
    def pendiente_centroides(self, img_bn,image):
        # #Operaciones morfologicas
        # SE_d = np.ones((5,5), np.uint8)
        # morf_d = cv2.dilate(img_bn, SE_d, iterations = 1)
        # SE_d2 = np.ones((4,4), np.uint8)
        # morf_d2 = cv2.dilate(morf_d, SE_d2, iterations = 2)
        # SE_d3 = np.ones((2,2), np.uint8)
        # morf_d3 = cv2.dilate(morf_d2, SE_d3, iterations = 2)
        SE_e = np.ones((5,5), np.uint8)
        morf_d3 = cv2.erode(img_bn, SE_e, iterations = 5)
        #self.imagenProcesada = morf_d3

        error = 0.0
        contornNear =0
        #Devuelve lista de contornos en formato (nPuntos,x,y)
        contours_blk, _ = cv2.findContours(morf_d3.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)	
        candidates = []
        
        num_contours = len(contours_blk)
        
        self.bandera.data = 0

        if num_contours > 0:  

            
            # Definir una lista de colores para los vértices
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # Rojo, Verde, Azul, Amarillo

            if num_contours > 0:
                for contour in contours_blk:
                    blackbox = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(blackbox)

                    # Encontrar el vértice con el menor valor en y
                    min_y = float('inf')
                    min_point = None
                    for i, (x_box, y_box) in enumerate(box):
                        if y_box < min_y:
                            min_y = y_box
                            min_point = (int(x_box), int(y_box))
                            min_color = colors[i % len(colors)]  # Usar colores de la lista en ciclo
                    
                    # Dibujar solo el vértice con el menor valor en y
                    if min_point is not None:
                        cv2.circle(image, min_point, 10, min_color, 3)
                
                # Dibujar todos los contornos
                cv2.drawContours(image, contours_blk, -1, (0, 0, 255), 3)
            
        
        self.imagenProcesada = image
        
        return error     
        

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
