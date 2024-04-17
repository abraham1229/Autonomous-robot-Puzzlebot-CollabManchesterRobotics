import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Vector, Path   # type: ignore
from geometry_msgs.msg import Twist
import math

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        self.pub_time_path = self.create_publisher(Float32, 'path_time', 1000)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Controller node initialized')
        self.msg = Vector()

        #Se hacen las suscripciones pertinentes
        self.subscription_odometria = self.create_subscription(
            Vector,
            'odometria',
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

        self.bandera = 1       

    #Lee los datos de odometría
    def signal_callback1(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta

    #Lee los datos del nodo de la llanta izquierda
    def signal_callback2(self, msg):
        if msg is not None and self.bandera == 1:
            self.x1 = msg.x1
            self.y1 = msg.y1
            self.x2 = msg.x2
            self.y2 = msg.y2
            self.x3 = msg.x3
            self.y3 = msg.y3
            self.x4 = msg.x4
            self.y4 = msg.y4


    def timer_callback(self):
        #Se crea el tipo de mensaje para mandar a cmd_vel
        self.twist_msg = Twist()

        #Primer trazo
        # if self.bandera == 0:
        #     self.angulo = math.atan2(self.y1,self.x1)
        #     self.velL = 0.0
        #     self.velA = 0.1
        #     if self.Postheta >= self.angulo:
        #         self.velA = 0.0
        #         self.velL = 0.1
        #         self.bandera = 1
            # else:
            #     self.velL = 0.1
            #     self.velA = 0.0
            #     self.bandera = 1
        self.velA = 0.0
        self.velL = 0.1    
        if self.Posx >= self.x1 and self.bandera == 1:
            self.velL = 0.0
            self.velA = 0.1
            self.angulo = round(math.atan2((self.y2-self.y1),(self.x2-self.x1)),2)
            if self.Postheta >= 1.5:
                self.velA = 0.0
                self.velL = 0.1
                self.bandera = 2
        #Segundo trazo
        elif self.Posy >= self.y2 and self.bandera == 2:
            self.velL = 0.0
            self.velA = 0.1
            self.angulo = round(math.atan2((self.y3-self.y2),(self.x3-self.x2)),2)
            if self.Postheta >= self.angulo:  
                self.velA = 0.0
                self.velL = 0.1
                self.bandera = 3
        #Tercer trazo
        elif self.Posx <= self.x3 and self.bandera == 3:
            self.velL = 0.0
            self.velA = 0.1
            self.angulo = round(math.atan2((self.y4-self.y3),(self.x4-self.x3)),2)
            if self.angulo < 0:
                self.angulo = self.angulo + 6
            if self.Postheta >= self.angulo:
                self.velA = 0.0
                self.velL = 0.1
                self.bandera = 4
        #Cuarto trazo
        elif self.Posy <= self.y4 and self.bandera == 4:
            self.velL = 0.0
            self.velA = 0.0
            # if self.Postheta >= 6.0:
            #     self.velA = 0.0
            #     self.velL = 0.0

        
        self.twist_msg.linear.x = self.velL
        self.twist_msg.angular.z = self.velA
        tiempo_trayectoria = Float32()
        tiempo_trayectoria.data = self.angulo
        self.pub_cmd_vel.publish(self.twist_msg)
        self.pub_time_path.publish(tiempo_trayectoria)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()