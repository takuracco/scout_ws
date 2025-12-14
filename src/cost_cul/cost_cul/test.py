import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class Listener(Node):
    def __init__(self):
        # 引数node_nameにlistenerを渡す。
        super().__init__("listener")

        # 引数msg_type、topic_name、callbackに渡してSubscriptionを作成
        # self.create_subscription(Float32MultiArray, '/roughness', self.callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_call, 10)

    # 処理用callback関数
    def callback(self, msg):
        print(msg.data)

    def odom_call(self, odom):
        self.get_logger().info(f"{odom.pose.pose.position.x}, {odom.pose.pose.position.y}")


def main():
    # RCLの初期化を実行
    rclpy.init()

    # Listenerクラスのコンスタンス化
    node = Listener()

    # ループに入りnode内の処理を実行させる
    rclpy.spin(node)

    # nodeを破壊
    node.destroy_node()

    # RCLをシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()