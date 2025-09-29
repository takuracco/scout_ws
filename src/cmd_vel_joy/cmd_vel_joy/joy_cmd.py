import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class joy_cmd(Node):
    def __init__(self):
        super().__init__('omuni_node_q')

        self.cmd_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.joy = None

    def joy_callback(self, joy_msg):
        self.joy = joy_msg

        self.set_cmd()

    def set_cmd(self):
        cmd_msg = Twist()

        cmd_msg.linear.x = self.joy.axes[1] * 5
        # cmd_msg.linear.y = self.joy.axes[1]
        cmd_msg.angular.z = self.joy.axes[3] * 5

        self.cmd_pub.publish(cmd_msg)

def main(args = None):
    rclpy.init(args = args)
    node = joy_cmd()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()