import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Path   # type: ignore

class My_Talker_Params(Node):
    def __init__(self):
        super().__init__('Path_generator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type', rclpy.Parameter.Type.INTEGER),
                ('velocidad_lineal', rclpy.Parameter.Type.DOUBLE),
                ('velocidad_angular', rclpy.Parameter.Type.DOUBLE),
            ])
        
        self.pub = self.create_publisher(Path, 'path_generator', 1000)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Path generator node initialized')
        self.msg = Path()
        self.velocidaLineal = 0.0
        self.velocidaAngular = 0.0

    def timer_callback(self):
        ri_type = self.get_parameter('type').get_parameter_value().integer_value
        self.msg.vel_lineal = self.get_parameter('velocidad_lineal').get_parameter_value().double_value
        self.msg.vel_angular = self.get_parameter('velocidad_angular').get_parameter_value().double_value
        
        if ri_type == 0:
            self.msg.x1 = 0.0
            self.msg.y1 = 0.0
            self.msg.x2 = 0.0
            self.msg.y2 = 0.0
            self.msg.x3 = 0.0
            self.msg.y3 = 0.0
            self.msg.x4 = 0.0
            self.msg.y4 = 0.0

        if ri_type == 1:
            # Triángulo
            self.msg.x1 = 0.5
            self.msg.y1 = 0.87
            self.msg.x2 = 1.0
            self.msg.y2 = 0.0
            self.msg.x3 = 0.0
            self.msg.y3 = 0.0
            self.msg.x4 = 0.0
            self.msg.y4 = 0.0
            self.msg.x5 = 0.0
            self.msg.y5 = 0.0
            self.msg.x6 = 0.0
            self.msg.y6 = 0.0

        if ri_type == 2:
            # Cuadrado
            self.msg.x1 = 1.0
            self.msg.y1 = 0.0
            self.msg.x2 = 1.0
            self.msg.y2 = 1.0
            self.msg.x3 = 0.0
            self.msg.y3 = 1.0
            self.msg.x4 = 0.0
            self.msg.y4 = 0.0
            self.msg.x5 = 0.0
            self.msg.y5 = 0.0
            self.msg.x6 = 0.0
            self.msg.y6 = 0.0

        if ri_type == 3:
            # Pentágono
            self.msg.x1 = 0.81
            self.msg.y1 = 0.59
            self.msg.x2 = 1.0
            self.msg.y2 = 0.0
            self.msg.x3 = 0.19
            self.msg.y3 = 0.59
            self.msg.x4 = 0.0
            self.msg.y4 = 0.0
            self.msg.x5 = 0.5
            self.msg.y5 = 1.0
            self.msg.x6 = 0.0
            self.msg.y6 = 0.0

        if ri_type == 4:
            # Hexágono
            self.msg.x1 = 0.87
            self.msg.y1 = 0.5
            self.msg.x2 = 1.0
            self.msg.y2 = 0.25
            self.msg.x3 = 0.87
            self.msg.y3 = 0.0
            self.msg.x4 = 0.13
            self.msg.y4 = 0.0
            self.msg.x5 = 0.0
            self.msg.y5 = 0.25
            self.msg.x6 = 0.13
            self.msg.y6 = 0.5

        

        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = My_Talker_Params()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
