import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Float32
from msgs_clase.msg import Vector, Path,Signal  # type: ignore
from geometry_msgs.msg import Twist
import math
import time

from yolov8_msgs.msg import InferenceResult # type: ignore
from yolov8_msgs.msg import Yolov8Inference # type: ignore


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
            Signal,
            '/signal_bool',
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


        self.senialesBool = Signal()

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
            self.senialesBool = msg
            

    # Callback del temporizador para controlar el movimiento del robot
    def timer_callback_controller(self):


        # Se hace una aceptación de error para que no oscile
        if self.errorLineal >= -0.05 and self.errorLineal <= 0.05:
            self.velA = 0.0
        else:
            self.velA = self.errorLineal * self.kpTheta

        
        # Se acota el límite de velocidad
        if self.velA > 0.2:
            self.velA = 0.2

        # No avanza hasta que detecte algún error 
        if self.errorLineal == 0.0:
            self.velA = 0.0
            self.velL = 0.0
        else:
            self.velL = 0.1
        

         #Condiciones de detección de seniales
        if self.senialesBool.ahead_only:
            pass
        
        elif self.senialesBool.turn_right:
            pass

        elif self.senialesBool.turn_left:
            pass

        elif self.senialesBool.roundabout:
            pass

        elif self.senialesBool.give_way:
            pass

        elif self.senialesBool.roadwork:
            pass

        elif self.senialesBool.stop:
            self.velA = 0.0
            self.velL = 0.0

        elif self.senialesBool.red_light:
            self.velA = 0.0
            self.velL = 0.0

        elif self.senialesBool.yellow_light:
            self.velL = self.velL/2

        elif self.senialesBool.green_light:
            pass

        elif self.senialesBool.dot_line:
            pass
        
        
        # Crear el mensaje Twist y publicarlo
        twist_msg = Twist()
        twist_msg.linear.x = self.velL
        twist_msg.angular.z = self.velA
        self.pub_cmd_vel.publish(twist_msg)



def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
