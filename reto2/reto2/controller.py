import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Vector, Path   # type: ignore
from geometry_msgs.msg import Twist
import math


class My_Talker_Params(Node):
    def __init__(self):
        super().__init__('Controller')
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Controller node initialized')
        self.msg = Vector()

        #Se hacen las suscripciones pertinentes
        self.subscription_odometria = self.create_subscription(
            Vector,
            'ri_Odometria',
            self.signal_callback1,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos
        
        #Se hacen las suscripciones pertinentes
        self.subscription_path = self.create_subscription(
            Path,
            'path_generator',
            self.signal_callback2,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos


        #Se declaran las variables para las coordenadas de la trayectoria
        self.x1 = 0.0
        self.y1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.x3 = 0.0
        self.y3 = 0.0
        self.x4 = 0.0
        self.y4 = 0.0

        #Se declaran las variables para las velocidades
        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0

        #Se declaran las velocidades
        self.velL = 0.0
        self.velA = 0.0

    
    #Lee los datos del nodo de la llanta izquierda
    def signal_callback1(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta

    #Lee los datos del nodo de la llanta izquierda
    def signal_callback2(self, msg):
        if msg is not None:
            self.x1 = msg.x1
            self.y1 = msg.y1


    def timer_callback(self):
        twist_msg = Twist()
        self.velL = 0.1
        self.velA = 0.0
        if self.Posx >= self.x1:
            self.velL = 0.0
        
        
        twist_msg.linear.x = self.velL
        twist_msg.angular.z = self.velA
        self.pub_cmd_vel.publish(twist_msg)




def main(args=None):
    rclpy.init(args=args)
    m_t_p = My_Talker_Params()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
