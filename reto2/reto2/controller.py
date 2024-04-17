import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Vector, Path   # Asegúrate de que el mensaje Path esté definido correctamente en msgs_clase.msg
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

        # Se hacen las suscripciones pertinentes
        self.subscription_odometria = self.create_subscription(
            Vector,
            'odometria',
            self.signal_callback1,
            rclpy.qos.qos_profile_sensor_data )  # Se debe de incluir la lectura de datos
        
        # Se hacen las suscripciones pertinentes
        self.subscription_path = self.create_subscription(
            Path,
            'path_generator',
            self.signal_callback2,
            rclpy.qos.qos_profile_sensor_data )  # Se debe de incluir la lectura de datos


        # Se declaran las variables para las coordenadas de la trayectoria
        self.trayectoria = []
        self.nPunto = 0
        
        # Se declaran las variables para las velocidades
        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0

        # Se declaran las velocidades
        self.velL = 0.0
        self.velA = 0.0

    # Lee los datos del nodo de la odometría
    def signal_callback1(self, msg):
        if msg is not None:
            self.Posx = msg.x
            self.Posy = msg.y
            self.Postheta = msg.theta

    # Lee los datos del nodo del generador de trayectorias
    def signal_callback2(self, msg):
        if msg is not None:
            self.trayectoria = [(msg.x1, msg.y1), (msg.x2, msg.y2), (msg.x3, msg.y3), (msg.x4, msg.y4)]

    def timer_callback(self):
        self.twist_msg = Twist()
        
        # Verifica si hay puntos en la trayectoria
        if self.trayectoria:
            # Si hay puntos en la trayectoria y no hemos alcanzado el último punto
            if self.nPunto < len(self.trayectoria):
                punto_meta = self.trayectoria[self.nPunto]
                punto_x, punto_y = punto_meta

                angulo_meta = math.atan2(punto_y, punto_x)

                

                # Normaliza el ángulo de diferencia entre -pi y pi para asegurar el menor giro
                angulo_diferencia = math.atan2(math.sin(angulo_meta), math.cos(angulo_meta))
                distancia_meta = math.sqrt(math.pow(punto_x,2)+math.pow(punto_y,2))
                distancia_actual = math.sqrt(math.pow(self.Posx,2)+math.pow(self.Posy,2))

                # Establece las velocidades angulares para girar hacia el punto
                self.velL = 0.0
                self.velA = 0.2 if angulo_diferencia > 0 else -0.2

                # Si el ángulo hacia el punto es pequeño, avanza hacia él
                if abs(angulo_diferencia) < 0.1:
                    self.velL = 0.2
                    self.velA = 0.0

                # Si estamos lo suficientemente cerca del punto, pasa al siguiente punto
                if distancia_actual >= distancia_meta:
                    self.nPunto += 1

            # Si hemos alcanzado el último punto en la trayectoria, detén el robot
            else:
                self.velL = 0.0
                self.velA = 0.0

        # Publica las velocidades calculadas
        self.twist_msg.linear.x = self.velL
        self.twist_msg.angular.z = self.velA
        self.pub_cmd_vel.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
