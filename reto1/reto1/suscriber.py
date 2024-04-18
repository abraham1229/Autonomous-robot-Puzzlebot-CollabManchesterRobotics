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
        super().__init__('ri_Odometria')
        #Se hacen las suscripciones pertinentes
        self.subscription_signal1 = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.signal_callback1,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos
        self.subscription_signal2 = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.signal_callback2,
            rclpy.qos.qos_profile_sensor_data)
        #Se crea el publicador que mandará mensaje personalizado
        self.pub_info = self.create_publisher(Vector, 'odometria', 1000)
        # Período de temporizador para 10Hz
        timer_period = 0.1 
        #Se declara el timer que llamará al callback
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #Se despliega en pantalla que se ha inicializado el nodo
        self.get_logger().info('Talker params node initialized')
        #Variables de velocidad
        self.velI = 0.0 #Variable de lectura izquierda de la llanta
        self.velD = 0.0 #Variable de lectura derecha de la llanta
        #Se define el theha 
        self.theha = 0.0
        #Valor del radio de la llanta
        self.r = 0.05
        #Valor de la distancia entre llantas
        self.l = 0.19
        #Se declaran variables a usar en el mensaje personalizado
        self.velocidadTheha = 0.0
        self.posX = 0.0
        self.posY = 0.0

        #Se define el periodo de los nodos
        self.periodo_lectura = 0.1

        #Variable para obtener las velocidades del robot
        self.velLineal = 0.0

        #Variable para detectar si se ha cambiado de signo
        self.cambioSingo = 0

        
    #Lee los datos del nodo de la llanta izquierda
    def signal_callback1(self, msg):
        if msg is not None:
            self.velI = msg.data

    #Lee los datos del nodo de la llanta derecha
    def signal_callback2(self, msg):
        if msg is not None:
            self.velD = msg.data

    
    #Se hace callback en el que se calcula la posición en x,y y theha
    def timer_callback(self):


        #Publicar los parámetros obtenidos con ayuda de msg personalizado.
        msgDato = Vector()

        #Se calcula thetha punto
        self.velocidadTheha = self.r*((self.velD-self.velI)/self.l)
        #Se calcula argumento velocidad
        self.velLineal = self.r*((self.velD+self.velI)/2)

        if self.cambioSingo == 0:
            if self.theha >= 3.14:
                self.theha = -3.14
                self.cambioSingo = 1
            else:
                self.theha += self.velocidadTheha*self.periodo_lectura
        
        elif self.cambioSingo == 1:
            if self.theha >= -0.5:
                self.theha = 0.0
                self.cambioSingo = 0
            else:
                self.theha += self.velocidadTheha*self.periodo_lectura
                



        self.posX += self.velLineal*math.cos(self.theha) *self.periodo_lectura
        self.posY += self.velLineal*math.sin(self.theha) *self.periodo_lectura
        
        #Se declaran en el mensaje costumizado
        msgDato.x = self.posX
        msgDato.y = self.posY
        msgDato.theta = self.theha
        #Se publica el dato
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
