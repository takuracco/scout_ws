import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class joy_cmd(Node):
    def __init__(self):
        super().__init__('omuni_node_q')

        self.cmd_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        self.odom_sub = self.create_subscription(Odometry,'/diff_drive_controller/odom',self.odom_callback , 10)

        self.joy = None

    def joy_callback(self, joy_msg):
        self.joy = joy_msg

        self.set_cmd()

    def set_cmd(self):
        cmd_msg = Twist()

        cmd_msg.linear.x = self.joy.axes[1] * 10
        # cmd_msg.linear.y = self.joy.axes[1]
        cmd_msg.angular.z = self.joy.axes[3] * 5

        self.cmd_pub.publish(cmd_msg)

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        odom_yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        self.get_logger().info(f"{odom_x},{odom_y},{odom_yaw},{norm}")

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