import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class SimpleCommander(Node):
    
    def __init__(self):
        super().__init__('simple_commander')
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.timer = self.create_timer(1, self.on_timer_tick)
    
    def on_timer_tick(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info('Published a message')

def main(args = None):
    rclpy.init(args=args)
    commander = SimpleCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
