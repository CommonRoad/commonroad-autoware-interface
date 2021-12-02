import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Cr2Auto(Node):

    def __init__(self):
        super().__init__('cr2autoware')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.declare_parameter('map_osm_file', '')
        self.map_filename = self.get_parameter('map_osm_file').get_parameter_value().string_value

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
        self.get_logger().info('map_osm_file: "%s"' % map_filename)


def main(args=None):
    rclpy.init(args=args)

    cr2auto = Cr2Auto()

    rclpy.spin(cr2auto)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cr2auto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
