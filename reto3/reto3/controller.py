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
        self.distancia = 0.0
        self.angulo_objetivo = 0.0

    # Callback para recibir la posición actual del robot
    def signal_callback1(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta

    # Callback para recibir los puntos de la trayectoria
    def signal_callback2(self, msg):
        if msg is not None:
            self.trayectoria = [(0,0),(msg.x1, msg.y1), (msg.x2, msg.y2), (msg.x3, msg.y3), (msg.x4, msg.y4)]

    # Callback del temporizador para controlar el movimiento del robot
    def timer_callback(self):


        # Verificar si hay puntos en la trayectoria
        if not self.trayectoria:
            self.get_logger().warn('No hay puntos en la trayectoria')
            return
        
        if self.indice_punto_actual >= len(self.trayectoria)-1:
                self.get_logger().info('Se alcanzó el final de la trayectoria')
                # Detener el robot
                self.velL = 0.0
                self.velA = 0.0
                return
        
        # Obtener las coordenadas del punto actual en la trayectoria
        target_x, target_y = self.trayectoria[self.indice_punto_actual+1]

        target_x_ant, target_y_ant = self.trayectoria[self.indice_punto_actual]
        
        

        # Calcular las coordenadas polares del punto objetivo
        self.distancia = math.sqrt((target_x - self.Posx)**2 + (target_y - self.Posy)**2)
        self.angulo_objetivo = math.atan2(target_y - target_y_ant, target_x - target_x_ant)


        
        if self.angulo_objetivo < self.Postheta+0.15 and self.angulo_objetivo > self.Postheta-0.15:
            self.velA = 0.0
            self.velL = 0.0

            if self.distancia > 0.1:
                self.velA = 0.0
                self.velL = 0.1
            else:

                self.indice_punto_actual += 1
        else:
            self.velA = 0.1
            self.velL = 0.0

        self.get_logger().info(f'------------------------------------------------------------')
        self.get_logger().info(f'Angulars meta y odo ({self.angulo_objetivo}, {self.Postheta})')
        self.get_logger().info(f'Distancias y punto ({self.distancia}, {self.indice_punto_actual})')



        


        
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