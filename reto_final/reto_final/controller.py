import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Float32
from msgs_clase.msg import Vector,Signal  # type: ignore
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
        
        self.subscription_seniales_detectadas = self.create_subscription(
            Signal,
            '/signal_bool',
            self.deteccion_callback,
            rclpy.qos.qos_profile_sensor_data )
        
        self.subscription_odometry = self.create_subscription(
            Vector,
            'odometria',
            self.odometry_callback,
            rclpy.qos.qos_profile_sensor_data )
        
        # Velocidades que se le darán al robot
        self.velL = 0.0
        self.velA = 0.0

        # Variable para leer el error del nodo line_error
        self.errorLinea = 0.0

        #Variables para el control
        #Theta 
        self.kpTheta = 0.21
        self.kdTheta = 0.015
        self.errorPrevio = 0.0

        # Tipo de mensaje para tener seniales detectadas
        self.senialesBool = Signal()
        self.cruce = False
        self.senialCruce = False
        self.numDeteccionesStop = 0

        #Se hace la detección del error
        self.lecturaError = False

        # Variables para almacenar la posición actual del robot
        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0


        # Valores en tiempod de detección de odometría
        self.distancia_actual = 0.0
        self.angulo_actual = 0.0
        self.distancia_deseado = 0.0
        self.angulo_deseado = 0.0

        # Mensaje de que el nodo ha sido inicializado
        self.get_logger().info('Controller node initialized')


     # Callback para recibir error del centro de la línea
    def line_error_callback(self, msg):
        #Si el dato es diferente a cero se guarda
        if msg is not None:
            self.errorLinea = msg.data
            self.lecturaError = True
        else:
            self.lecturaError = False

    def deteccion_callback(self, msg):
        #Si el dato es diferente a cero se guarda
        if msg is not None:
            self.senialesBool = msg
    
    # Callback para recibir la posición actual del robot
    def odometry_callback(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta
            

    # Callback del temporizador para controlar el movimiento del robot
    def timer_callback_controller(self):

        # Se hace la detección si es que está en un cruce
        if self.senialesBool.dot_line and not self.cruce:
            if self.numDeteccionesStop == 0:
                self.numDeteccionesStop = 1
            else:
                self.cruce = True
                self.senialCruce = False
                self.distancia_actual = self.Posx
                self.angulo_actual = self.Postheta
                self.numDeteccionesStop = 0

        # En el caso de que se haya detectado la línea punteada:
        if self.cruce:
            if not self.senialCruce:
            
                #Condiciones necesarias cuando tenga cruces
                if self.senialesBool.ahead_only:
                    self.distancia_deseado = 0.3
                    self.angulo_deseado = 0.0
                    self.senialCruce = True

                elif self.senialesBool.turn_right:
                    self.distancia_deseado = 0.3
                    self.angulo_deseado = -1.5
                    self.senialCruce = True

                # Izquierda debe de ser positivo
                elif self.senialesBool.turn_left:
                    self.distancia_deseado = 0.3
                    self.angulo_deseado = 1.5
                    self.senialCruce = True

                elif self.senialesBool.roundabout:
                    self.distancia_deseado = 0.3
                    self.angulo_deseado = 3.14
                    self.senialCruce = True

                elif self.senialesBool.give_way:
                    self.distancia_deseado = 0.3
                    self.angulo_deseado = 0.0
                    self.senialCruce = True

                else:
                    self.velA = 0.0
                    self.velL = 0.0
                    self.senialCruce = False
            
            else:
                
                if (self.Posx - self.distancia_actual) < self.distancia_deseado:
                    self.velL = 0.1
                    self.velA = 0.0

                else:
                    errorTheta = self.Postheta - self.angulo_actual

                    if self.angulo_deseado > 0:
                        if errorTheta < self.angulo_deseado:
                            self.velL = 0.0
                            self.velA = 0.1
                        else:
                            self.cruce = False
                            self.senialCruce = False
                    
                    elif self.angulo_deseado == 0:
                        self.cruce = False
                        self.senialCruce = False

                    else:
                        if errorTheta > self.angulo_deseado:
                            self.velL = 0.0
                            self.velA = -0.1
                        else:
                            self.cruce = False
                            self.senialCruce = False
                

                    
        else:
            
            
            # Se hace una aceptación del error para que no oscile
            if self.errorLinea >= -0.05 and self.errorLinea <= 0.05:
                self.velA = 0.0
            else:
                # Calcular el término proporcional
                proportional = self.kpTheta * self.errorLinea

                # Calcular el término derivativo
                derivative = self.kdTheta * (self.errorLinea - self.errorPrevio)

                # Actualizar el error previo
                self.errorPrevio = self.errorLinea

                # Calcular la señal de control
                self.velA = proportional + derivative

            #Se acota la velocidad positiva
            if self.velA > 0.2:
                self.velA = 0.2

            #Se acota la velocidad negativa
            if self.velA < -0.2:
                self.velA = -0.2    

            # No avanza hasta que lea del tópico error 
            if self.lecturaError:
                self.velL = 0.08
            
            else:
                self.velL = 0.00
                self.velA = 0.00
            

        #Condiciones de detección de señales generales
        
        if self.senialesBool.roadwork:
            self.velL = 0.04

        # Manejo de la señal de stop
        elif self.senialesBool.stop:
            self.velA = 0.0
            self.velL = 0.0

        elif self.senialesBool.red_light:
            self.velA = 0.0
            self.velL = 0.0

        elif self.senialesBool.yellow_light:
            self.velL = 0.04

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
