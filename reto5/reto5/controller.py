import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Float32
from msgs_clase.msg import Vector, Path   # type: ignore
from geometry_msgs.msg import Twist
import math


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


        # Velocidades que se le darán al robot
        self.velL = 0.02
        self.velA = 0.0
        # Variable para leer el error del nodo line_error
        self.errorLineal = 0.0

        #Variables para el control
        #Theta 
        self.kpTheta = 0.15


        # Mensaje de que el nodo ha sido inicializado
        self.get_logger().info('Controller node initialized')


     # Callback para recibir error del centro de la línea
    def line_error_callback(self, msg):
        #Si el dato es diferente a cero se guarda
        if msg is not None:
            self.errorLineal = msg.data

    # Callback del temporizador para controlar el movimiento del robot
    def timer_callback_controller(self):
        
        

        if self.errorLineal >= -0.05 and self.errorLineal <= 0.05:
            self.velA = 0.0
        
        else:
            self.velA = self.errorLineal * self.kpTheta

        
        if self.velA > 0.1:
            self.velA = 0.1

        
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



def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()