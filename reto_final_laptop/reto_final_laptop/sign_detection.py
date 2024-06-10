from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from msgs_clase.msg import Signal,Dotline   # type: ignore

from yolov8_msgs.msg import InferenceResult # type: ignore
from yolov8_msgs.msg import Yolov8Inference # type: ignore

import time

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        #self.model = YOLO('/home/abraham/modelos/DeteccionSeniales4_340.pt')
        self.model = YOLO('/home/abraham/modelos/versionDosModelos.pt')

        self.yolov8_inference = Yolov8Inference()
        # Realiza la suscripción de la imágen
        self.subscription = self.create_subscription(
            Image,
            '/sign_information',
            self.camera_callback,
            10) 
        
        # Realiza la suscripción a la línea vertical
        self.subscription = self.create_subscription(
            Dotline,
            '/vertical_bool',
            self.vertical_callback,
            10) 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.predi_pub = self.create_publisher(Signal, "/signal_bool", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        
        self.valoresObtenidos = []

        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback_signs)

        self.img = np.ones((480, 640, 3), dtype=np.uint8)

        # Variables para gestionar la señal de stop
        self.stop_detected_time = None
        self.stop_signal_sent = False

        # Variables para detectar el dotline
        self.dot_line_detected_time = None
        self.dot_line_sent = False

        # Se pone el tipo de mensaje que va leer el tipo de vertical
        self.lineaRight = False
        self.lineaLeft = False
        self.lineaBoth = False

        
        self.get_logger().info('Sign detection node initialized')

    def camera_callback(self, data):
        if data is not None:
            self.img = bridge.imgmsg_to_cv2(data, "bgr8")
    
    def vertical_callback(self, data):
        if data is not None:
            self.lineaBoth = data.both
            self.lineaRight = data.right
            self.lineaLeft = data.left
    

    def timer_callback_signs(self):
        results = self.model(source=self.img, conf=0.4, verbose=False)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[1])
                self.inference_result.left = int(b[0])
                self.inference_result.bottom = int(b[3])
                self.inference_result.right = int(b[2])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        #self.img_pub.publish(img_msg)
        #self.yolov8_pub.publish(self.yolov8_inference)

        self.escribirMensaje(self.yolov8_inference)

        self.predi_pub.publish(self.senialesDetectadas)

        self.yolov8_inference.yolov8_inference.clear()
        self.valoresObtenidos.clear()

    def escribirMensaje(self, yoloInference):
        # Se crea el mensaje a publicar.
        self.senialesDetectadas = Signal()
        # Variables para calcular la senial más cercana.
        max_area = 0
        signal_with_max_area = None

        # Por cada inferencia que se tuvo se toman decisiones dependiendo
        # de lo que detecto.
        for inference in yoloInference.yolov8_inference:
            class_name = inference.class_name
            nearest = inference.bottom

             # Se pone en un alta priorida el semaforo.
            if class_name == "greenLight":
                self.get_logger().info(f'{nearest})')
                if nearest> 105: 
                    self.senialesDetectadas.green_light = True
            elif class_name == "redLight":
                self.get_logger().info(f'{nearest})') 
                if nearest > 105:
                    self.senialesDetectadas.red_light = True
                    return
                
            elif class_name == "yellowLight":
                self.get_logger().info(f'{nearest})')
                if nearest > 105:
                    self.senialesDetectadas.yellow_light = True
            
            # Si es dotLine se manda el mensaje para saber que se encuentra en 
            # un cruce.
            if class_name == "dotLine":
                print(nearest)
                if nearest > 195: # 200
                    if self.senialesDetectadas.yellow_light:
                        self.senialesDetectadas.yellow_light = False
                        self.senialesDetectadas.red_light = True
                        return
                    
                    else:
                        
                        if not self.dot_line_detected_time:
                            self.dot_line_detected_time = time.time()
                            self.senialesDetectadas.dot_line = True
                            self.dot_line_sent = True

                        else:
                            
                            elapsed_time = time.time() - self.dot_line_detected_time
                
                            if elapsed_time < 3.0:
                                self.senialesDetectadas.dot_line = True

                            elif elapsed_time >= 15.0:
                                self.dot_line_sent = False
                                self.dot_line_detected_time = None
            
            # Si la inferencia es diferente a un tipo de línea punteda se calcula
            # cual es el más cercano
            if class_name != "dotLine":
                if nearest > max_area:
                        max_area = nearest
                        signal_with_max_area = inference
            


        # En el caso de que se haya encontrado senial se mandan como true solamente
        # si están a cierta distancia. (dependiendo al altura de cada senial se calcula la distancia en la que está)
        if signal_with_max_area:
            
            class_name = signal_with_max_area.class_name
            if class_name == "aheadOnly": 
                self.senialesDetectadas.ahead_only = True

            elif class_name == "giveWay":
                # Se mantiene en dotline por tres segundos para que se detenga y luego avance
                if self.dot_line_detected_time:
                    elapsed_time = time.time() - self.dot_line_detected_time
                    if elapsed_time > 3.0:
                        self.senialesDetectadas.give_way = True
                        self.senialesDetectadas.dot_line = False

            elif class_name == "roadwork": 
                if signal_with_max_area.bottom > 125:
                    self.senialesDetectadas.roadwork = True

            elif class_name == "roundabout":
                self.senialesDetectadas.roundabout = True

            elif class_name == "stop":  
                if signal_with_max_area.bottom > 125:
                    # Si detectamos "stop", verificamos el tiempo
                    if not self.stop_signal_sent:
                        self.stop_detected_time = time.time()
                        self.senialesDetectadas.stop = True
                        self.stop_signal_sent = True
                    else:
                        elapsed_time = time.time() - self.stop_detected_time
            
                        if elapsed_time < 5.0:
                            self.senialesDetectadas.stop = True

                        elif elapsed_time >= 12.0:
                            self.stop_signal_sent = False
                            self.stop_detected_time = None

            # Se guarda la dirección del giro dependiendo de el lado en el que detecto la línea.
            elif class_name == "turnLeft": 
                self.senialesDetectadas.turn_left = True
            elif class_name == "turnRight": 
                self.senialesDetectadas.turn_right = True
            
            # Se hace la condición de dirección de rondabout
            # para que siga el retorno
            if self.senialesDetectadas.roundabout:
                if self.lineaLeft:
                    self.senialesDetectadas.turn_left = True

                elif self.lineaRight:
                    self.senialesDetectadas.turn_right = True
                
                elif self.lineaBoth:
                    self.senialesDetectadas.turn_right = True


            if self.senialesDetectadas.turn_right or self.senialesDetectadas.turn_left:
                if self.lineaLeft:
                    self.senialesDetectadas.turn_left = True
                    self.senialesDetectadas.turn_right = False
                    
                elif self.lineaRight:
                    self.senialesDetectadas.turn_right = True
                    self.senialesDetectadas.turn_left = False




def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
