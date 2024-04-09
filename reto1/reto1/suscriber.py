import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Vector   # type: ignore
import math


#Se crea el nodo My_Talker_Params tomando objeto de Node.
class My_Talker_Params(Node):
    #Se inicializa el nodo
    def __init__(self):
        #Se crear el nodo que será encargado de publicar la señal
        super().__init__('ri_publisher')
        #Se hacen las suscripciones pertinentes
        self.subscription_signal1 = self.create_subscription(
            Float32,
            'senial1',
            self.signal_callback1,
            1000)
        self.subscription_signal2 = self.create_subscription(
            Float32,
            'senial2',
            self.signal_callback2,
            1000)
        #Se crea el publicador que mandará mensaje personalizado
        self.pub_info = self.create_publisher(Vector, 'ri_signal_params', 1000)  # Frecuencia de publicación de 1kHz
        # Período de temporizador para 10kHz
        timer_period = 0.01 
        #Se declara el timer que llamará al callback
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Se despliega en pantalla que se ha inicializado el nodo
        self.get_logger().info('Talker params node initialized')
        self.velI = 0.0
        self.velD = 0.0
        
    
    def signal_callback1(self, msg):
        if msg is not None:
            self.velI = msg.data

    def signal_callback2(self, msg):
        if msg is not None:
            self.velD = msg.data

          
    def timer_callback(self):
        #Publicar los parámetros obtenidos con ayuda de msg personalizado.
        msgDato = Vector()
        msgDato.x = self.velD
        msgDato.y = self.velI
        msgDato.z = 0.0
        self.pub_info.publish(msgDato)
        

#La función que será llamada según nuestro setup
def main(args=None):
    #Se inicializa el entorno de ros
    rclpy.init(args=args)
    #Se crea una instancia de la clase creada previamente.
    m_t_p = My_Talker_Params()
    #Creación del bucle de eventos
    rclpy.spin(m_t_p)
    #Se liberan recursos
    m_t_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
