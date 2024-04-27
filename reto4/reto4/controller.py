import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from msgs_clase.msg import Vector, Path   # type: ignore
from geometry_msgs.msg import Twist
import math


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        self.pub_type = self.create_publisher(Int32, 'path_type', 1000)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
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
            rclpy.qos.qos_profile_sensor_data )
        
        #Se hacen las suscripciones pertinentes
        self.subscription_light = self.create_subscription(
            Int32,
            'color_detection',
            self.signal_callback_traffic,
            rclpy.qos.qos_profile_sensor_data )

        # Variables para almacenar la posición actual del robot
        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0

        # Lista para almacenar los puntos de la trayectoria
        self.trayectoria = []

        # Índice del punto actual en la trayectoria
        self.indice_punto_actual = 0

        # Velocidades lineal y angular del robot
        self.velL = 0.0
        self.velA = 0.0
        self.error_distancia = 0.0
        self.angulo_objetivo = 0.0
        self.errorTheta = 0.0

        #Variables para el control
        #Theta 
        self.kpTheta = 0.2

        #Num lados
        self.numPuntos = 5
        self.msgType = Int32()
        self.type = 2


        self.color_traffic_light = 0

        self.msgType.data = self.type
        self.pub_type.publish(self.msgType)



    # Callback para recibir la posición actual del robot
    def signal_callback1(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta

    # Callback para recibir los puntos de la trayectoria
    def signal_callback2(self, msg):
        if msg is not None:
            self.trayectoria = [(0,0),(msg.x1, msg.y1), (msg.x2, msg.y2), (msg.x3, msg.y3), (msg.x4, msg.y4), (msg.x5, msg.y5), (msg.x6, msg.y6),(msg.x7, msg.y7),(msg.x8, msg.y8)]

     # Callback para recibir los puntos de la trayectoria
    def signal_callback_traffic(self, msg):
        if msg is not None:
            self.color_traffic_light = msg.data

    # Callback del temporizador para controlar el movimiento del robot
    def timer_callback(self):


        # Verificar si hay puntos en la trayectoria
        if not self.trayectoria:
            self.get_logger().warn('No hay puntos en la trayectoria')
            return
        
        if self.numPuntos >= 8:
                self.get_logger().info(f'Trayectoria alcanzada')
                # Detener el robot
                self.velL = 0.0
                self.velA = 0.0
                twist_msg = Twist()
                twist_msg.linear.x = self.velL
                twist_msg.angular.z = self.velA
                self.pub_cmd_vel.publish(twist_msg)
                return


        if self.indice_punto_actual >= len(self.trayectoria)-1 or self.indice_punto_actual > self.numPuntos:
                self.get_logger().info(f'Type ({self.type})')
                # Detener el robot
                self.velL = 0.0
                self.velA = 0.0
                twist_msg = Twist()
                twist_msg.linear.x = self.velL
                twist_msg.angular.z = self.velA
                self.pub_cmd_vel.publish(twist_msg)
                self.type += 1
                self.msgType.data = self.type
                self.pub_type.publish(self.msgType)
                self.numPuntos += 1
                self.indice_punto_actual = 0
                return
        
        # Obtener las coordenadas del punto actual en la trayectoria
        target_x, target_y = self.trayectoria[self.indice_punto_actual+1]

        target_x_ant, target_y_ant = self.trayectoria[self.indice_punto_actual]
        
        

        # Calcular las coordenadas polares del punto objetivo
        #Se calcula el error lineal
        self.error_distancia = math.sqrt((target_x - self.Posx)**2 + (target_y - self.Posy)**2)

        #Se calcula el error angular
        self.angulo_objetivo = math.atan2(target_y-target_y_ant, target_x-target_x_ant)
        self.errorTheta = self.angulo_objetivo - self.Postheta

        # Se deja en angulos de menos pi a pi
        if self.errorTheta >= math.pi:
            self.errorTheta -= 2 * math.pi
        elif self.errorTheta <= -math.pi:
            self.errorTheta += 2 * math.pi

        #Se aplican condiciones respecto al semaforoo

        self.velA = self.kpTheta*self.errorTheta

        if self.velA > 0.15:
            self.velA = 0.15


        if  self.color_traffic_light == 1:
            self.velL = 0.2
        elif self.color_traffic_light == 2:
            self.velL = 0.1
        elif self.color_traffic_light == 3 or self.color_traffic_light == 0 :
            self.velL = 0.0


        if self.errorTheta > 0.05 or self.errorTheta < -0.05:
            self.velL = 0.0
                

        if self.errorTheta < 0.05 and self.errorTheta > -0.05 and self.error_distancia < 0.05:
            self.indice_punto_actual += 1

        # self.get_logger().info(f'Lineal ({self.velL})')
        self.get_logger().info(f'Color ({self.color_traffic_light})')

        
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