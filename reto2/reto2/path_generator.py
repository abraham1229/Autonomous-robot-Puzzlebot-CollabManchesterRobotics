import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Path   # type: ignore
from scipy import signal
import math


#Se crea el nodo My_Talker_Params tomando objeto de Node.
class My_Talker_Params(Node):
    #Se inicializa el nodo
    def __init__(self):
        #Se crear el nodo que será encargado de publicar la señal
        super().__init__('Path_generator')
        #Se declara la lista de parámetros a utilizar
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type', rclpy.Parameter.Type.INTEGER)
            ])
        
        # Frecuencia de publicación de 10Hz
        self.pub = self.create_publisher(Float32, 'path_generator', 1000)  
        # Período de temporizador para 10Hz
        timer_period = 0.1 
        #Se declara el timer que llamará al callback
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Se notifica que ha sido creado correctamente
        self.get_logger().info('Path generator node initialized')
        self.msg = Path()

    def timer_callback(self):

        ri_type = self.get_parameter('ri_type').get_parameter_value().integer_value
        #Se generan condiciones para las 5 señales y lectura de sus params 
        if ri_type == 0: #Condicion del cuadrado
            self.msg.x1 = 1
            self.msg.y1 = 0
            self.msg.x2 = 1
            self.msg.y2 = 1
            self.msg.x3 = 0
            self.msg.y3 = 1
            self.msg.x4 = 0
            self.msg.y4 = 0

        elif ri_type == 1:
            self.msg.x1 = 1
            self.msg.y1 = 1
            self.msg.x2 = 4
            self.msg.y2 = 4
            self.msg.x3 = 2
            self.msg.y3 = 2
            self.msg.x4 = 0
            self.msg.y4 = 0

        elif ri_type == 2:
            self.msg.x1 = 0
            self.msg.y1 = 4
            self.msg.x2 = 5
            self.msg.y2 = 8
            self.msg.x3 = 2
            self.msg.y3 = 6
            self.msg.x4 = 0
            self.msg.y4 = 0

        elif ri_type == 3:
            self.msg.x1 = 0
            self.msg.y1 = 4
            self.msg.x2 = 0
            self.msg.y2 = 8
            self.msg.x3 = 2
            self.msg.y3 = 4
            self.msg.x4 = 0
            self.msg.y4 = 7
        else:
            self.msg.x1 = 1
            self.msg.y1 = 0
            self.msg.x2 = 1
            self.msg.y2 = 1
            self.msg.x3 = 0
            self.msg.y3 = 1
            self.msg.x4 = 0
            self.msg.y4 = 0

        # msg.data = signal_value
        self.pub.publish(self.msg)

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
