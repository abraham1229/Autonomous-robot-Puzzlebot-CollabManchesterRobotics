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
            
            self.pub_line_image.publish(self.bridge.cv2_to_imgmsg(self.imagenProcesada))
            #self.get_logger().info(f'{self.errorMsg.data})')
        else:
            self.get_logger().info(f'Failed to process image')


    def calculoError(self,img):
        self.resize_image(img)
        self.preprocess(self.imagenCortada)
        error = self.pendiente_centroides(self.imagenBN)
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
        self.imagenBN = imagen_binarizada
        
    # Función que calcula el error con los centroides
    def pendiente_centroides(self, img_bn):
        #Operaciones morfologicas
        SE_d = np.ones((10,10), np.uint8)
        morf_d = cv2.dilate(img_bn, SE_d, iterations = 5)
        SE_e = np.ones((10,10), np.uint8)
        morf_e = cv2.erode(morf_d, SE_e, iterations = 1)
        self.imagenProcesada = morf_e

        #Conteo de pixeles
        centro_img_x = int(morf_e.shape[1]/2)
        centro_img_y = 10
        dimy = int(morf_e.shape[0]/4)

        cont = 0
        cuadricula = 0
        
        bandera =centroide_primer_punto_x = 0
        centroide_primer_punto_y = 0
        for i in range (morf_e.shape[1]):
            if morf_e[dimy][i] == 0:
                cont += 1
            if (bandera == 0 and morf_e[dimy][i] == 0):
                centroide_primer_punto_x = i
                centroide_primer_punto_y = dimy
                bandera = 1
        error = ((centroide_primer_punto_x + cont/2)-centro_img_x )/(morf_e.shape[1])
	
        for i in range (morf_e.shape[1]):
            if morf_e[1][i] == 0:
                cuadricula  += 1
                
        
        if cuadricula == 0 and self.cuadricula_ant != cuadricula:
            if self.contador == 4:
                self.bandera.data = 1
            else: 
                self.contador += 1
            
            
        self.cuadricula_ant = cuadricula

        # Dibujar los centroides en la imagen binarizada
        img_points = cv2.cvtColor(img_bn, cv2.COLOR_GRAY2BGR)
        cv2.circle(img_points, (centro_img_x, centro_img_y), 5, (0, 255, 0), -1)
        cv2.circle(img_points, (centroide_primer_punto_x, centroide_primer_punto_y), 5, (0, 0, 255), -1)
        
        return error

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
