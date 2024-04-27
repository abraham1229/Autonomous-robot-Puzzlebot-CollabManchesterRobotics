import rclpy
from rclpy.node import Node
from std_msgs.msg import  Int32
from msgs_clase.msg import Path   # type: ignore

class My_Talker_Params(Node):
    def __init__(self):
        super().__init__('Color_detection')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('color', rclpy.Parameter.Type.INTEGER)
            ])
                
        
        self.pub = self.create_publisher(Int32 , 'color_detection', 1000)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Color detection node initialized')
        self.color = Int32()
        self.color = 0

    def timer_callback(self):
        colorParam = self.get_parameter('color').get_parameter_value().integer_value
        
        msg = Int32()
        msg.data = colorParam
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = My_Talker_Params()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
