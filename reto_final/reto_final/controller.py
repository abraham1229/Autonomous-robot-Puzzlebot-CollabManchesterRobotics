import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Float32
from msgs_clase.msg import Vector, Path   # type: ignore
from geometry_msgs.msg import Twist
import math
import time

from msgs_clase.msg import InferenceResult
from msgs_clase.msg import Yolov8Inference


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback_controller)
        
        #Se hacen las suscripciones pertinentes
        self.subscription_light = self.create_subscription(
            Float32,
            'error_line',
            self.line_error_callback,
            rclpy.qos.qos_profile_sensor_data )
        
        self.subscription_frenado = self.create_subscription(
            Int32,
            'frenado',
            self.frenado_callback,
            rclpy.qos.qos_profile_sensor_data )

        self.subscription_frenado = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.deteccion_callback,
            rclpy.qos.qos_profile_sensor_data )
        




        self.frenado = 0
        self.historial = 0
        # Velocidades que se le darán al robot
        self.velL = 0.02
        self.velA = 0.0
        # Variable para leer el error del nodo line_error
        self.errorLineal = 0.0

        #Variables para el control
        #Theta 
        self.kpTheta = 0.15


        # Variable de la inferencia de YOLO
        self.yolov8_inference = Yolov8Inference()
        self.valoresObtenidos = []


        # Mensaje de que el nodo ha sido inicializado
        self.get_logger().info('Controller node initialized')


     # Callback para recibir error del centro de la línea
    def line_error_callback(self, msg):
        #Si el dato es diferente a cero se guarda
        if msg is not None:
            self.errorLineal = msg.data

    def frenado_callback(self, msg):
        #Si el dato es diferente a cero se guarda
        if msg is not None:
            self.frenado = msg.data

    def deteccion_callback(self, msg):
        #Si el dato es diferente a cero se guarda
        if msg is not None:
            for inference in msg.yolov8_inference:
                self.valoresObtenidos.append(inference.class_name)
            

    # Callback del temporizador para controlar el movimiento del robot
    def timer_callback_controller(self):
        
        if self.frenado == 1:
            self.velA = 0.0
            self.velL = 0.0

        if self.errorLineal >= -0.05 and self.errorLineal <= 0.05:
            self.velA = 0.0
        
        else:
            self.velA = self.errorLineal * self.kpTheta

        
        if self.velA > 0.2:
            self.velA = 0.2

        
        if self.errorLineal == 0.0:
            self.velA = 0.0
            self.velL = 0.0
        else:
            self.velL = 0.1
            
        
        # Crear el mensaje Twist y publicarlo
        twist_msg = Twist()
        twist_msg.linear.x = self.velL
        twist_msg.angular.z = self.velA
        self.pub_cmd_vel.publish(twist_msg)



        # Manda las clases que obtiene
        for class_name in self.valoresObtenidos:
            if class_name == "person": 
                self.get_logger().info(f'Class names collected: {1}')
            else: 
                self.get_logger().info(f'Class names collected: {2}')


        self.valoresObtenidos.clear()



def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
