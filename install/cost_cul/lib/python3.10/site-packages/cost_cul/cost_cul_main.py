import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


from .grid_manage import grid_management
from .roughness_cost import roughness_cost
from .slope_cul import slope_cul
from .parameter import Parameter

class Pose:
    """構造体"""
    def __init__(self, x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0):
        self.x     = x
        self.y     = y
        self.z     = z
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw
        self.qu_x  = 0.0
        self.qu_y  = 0.0
        self.qu_z  = 0.0
        self.qu_w  = 0.0

    def get_rotation(self, x, y, z, w):
        self.qu_x = x
        self.qu_y = y
        self.qu_z = z
        self.qu_w = w

        # yaw (Z)
        self.yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        # roll (X)
        self.roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        # pitch (Y) ※asinの入力が丸め誤差で±1を超えることがあるのでclipする
        sinp = 2.0 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        self.pitch = np.arcsin(sinp)

class Cost_Node(Node):
    def __init__(self):
        super().__init__('cost_cul_node')

        # パラメータ（必要なら declare_parameter で外部指定可）
        param = Parameter()
        self.grid_size = param.grid_size
        self.cell_res  = param.cell_res
        self.radius_m  = param.radius_m

        # 初期化
        self.points = np.empty((0, 3), dtype=np.float32)
        # self.grid_var = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)

        # ロボット現在地（本当はTFやOdomで更新するはず。ここでは固定 or 別APIから更新）
        self.pose = Pose(param.robot_x, param.robot_y, param.robot_z, param.robot_roll, param.robot_pitch, param.robot_yaw)

        # class定義
        self.roughness = roughness_cost(self.get_logger())
        self.slopecost = slope_cul(self.get_logger())
        self.grid_manege = grid_management(self.get_logger())

        # Publisher / Subscriber
        self.sub = self.create_subscription(PointCloud2, '/scan', self.callback, 10)
        self.odomsub = self.create_subscription(Odometry, '/ground_truth/odom', self.odomcallback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/roughness_cost', 10)
        self.slope_pub = self.create_publisher(Float32MultiArray, '/slope_cost', 10)
        self.grid_pub = self.create_publisher(MarkerArray, "/grid", 10)


        self.get_logger().info('cost_cul node is up.')


    def callback(self, scan):
        """LiDARのデータの"""
        self.get_logger().info('scanデータ受信')
        self.grid_manege.get_point(scan, self.pose.x, self.pose.y, self.pose.z, self.pose.roll, self.pose.pitch, self.pose.yaw)
        points = self.grid_manege.Cell_point

        if points is None or points == 0:
            self.get_logger().warn("ポイントが 0 件")
            return

        self.points = points
        # self.get_logger().info(f"サンプル: {points[:500]}")
        self.grid_manege.grid_var = self.roughness.roughness_cul(self.points)
        self.grid_manege.grid_plane = self.slopecost.set_grid_plane(self.roughness.grid_plane)
        self.get_logger().info("ここまで完了")
        self.grid_manege.slope_cost = self.slopecost.all_slope_cul()
        # self.get_logger().info(f"slope_cost:{self.slope_cost}")
        self.publish_grid()
        self.publish_cost()

    def odomcallback(self, odom):
        """自己位置の処理"""
        self.pose.x = odom.pose.pose.position.x
        self.pose.y = odom.pose.pose.position.y
        self.pose.z = odom.pose.pose.position.z
        self.pose.get_rotation(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        

    def publish_cost(self):
        """pubするのをまとめる"""
        self.publish_roughness()
        self.publish_slope_cost()

    def publish_slope_cost(self):
        """slope_cost (dict: (i,j)->np.array(8)) を /slope_cost に配信"""
        slope = getattr(self.grid_manege, "slope_cost", None)
        if not slope:
            self.get_logger().warn("slope_cost が空です。")
            return

        data = []
        for (i, j), arr in slope.items():
            if arr is None:
                continue

            a = np.asarray(arr, dtype=np.float32).reshape(-1)
            if a.size != 8:
                # 想定外（デバッグ用）
                self.get_logger().warn(f"slope_cost[{(i,j)}] size={a.size} (expected 8)")
                continue

            # inf / nan を 0 にする（扱いは好みで）
            a = np.where(np.isfinite(a), a, 0.0).astype(np.float32)

            # 1セル = [i, j, c0..c7]
            data.extend([float(i), float(j)] + a.tolist())

        msg = Float32MultiArray()
        msg.data = data

        # 1レコード10要素（i,j,8dir）
        msg.layout.dim = [
            MultiArrayDimension(label="records(i,j,c0..c7)", size=len(data)//10, stride=len(data)),
            MultiArrayDimension(label="elements", size=10, stride=10),
        ]
        msg.layout.data_offset = 0

        self.slope_pub.publish(msg)

    def publish_roughness(self):
        """roughness.grid_var (dict: (i,j)->val) を /roughness に配信"""
        grid_var = getattr(self.roughness, "grid_var", None)
        if not grid_var:
            self.get_logger().warn("grid_var が空です。")
            return

        data = []
        for (i, j), v in grid_var.items():
            if v is None:
                continue
            v = float(v)
            if not np.isfinite(v):
                continue
            data.extend([float(i), float(j), v])  # 3つで1セット

        msg = Float32MultiArray()
        msg.data = data

        # layout は「triplets」で送ってることが分かる程度にしておく（任意）
        msg.layout.dim = [
            MultiArrayDimension(label="triplets(i,j,val)", size=len(data)//3, stride=len(data)),
            MultiArrayDimension(label="elements", size=3, stride=3),
        ]
        msg.layout.data_offset = 0

        self.pub.publish(msg)
        # self.get_logger().info(f"粗さ(map dict) Publish: cells={len(data)//3}")


    def publish_grid(self):
        """gridに出力（roughness を MarkerArray で可視化）"""
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()

        cell = float(self.cell_res)
        frame = "base_link"   # base_link 原点中心にローカル表示

        marker_id = 0

        # ロボットの現在セル（世界座標セルindex）
        now_i = int(np.floor(self.pose.x / self.cell_res))
        now_j = int(np.floor(self.pose.y / self.cell_res))
        self.get_logger().info(f"現在のセル{now_i},{now_j}")

        N = int(self.grid_size)
        half = N / 2.0

        # ---------- CUBE（セル） ----------
        for i in range(N):
            for j in range(N):
                # base_link周りのローカルセル(i,j)が対応する「世界セルキー」
                gi = i + now_i - (N // 2)
                gj = j + now_j - (N // 2)
                key = (gi, gj)

                v = self.grid_manege.grid_var.get(key, None)

                unknown = (v is None) or (not np.isfinite(v)) or (v < 0.0)

                if unknown:
                    std = 0.0
                else:
                    std = float(np.sqrt(v))
                    std = min(std, 10.0)  # 上限クリップ

                marker = Marker()
                marker.header.stamp = now
                marker.header.frame_id = frame
                marker.ns = "roughness_grid"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.scale.x = cell
                marker.scale.y = cell
                marker.scale.z = 0.02

                # ローカル表示：base_link中心を(0,0)として配置
                marker.pose.position.x = (i - half + 0.5) * cell
                marker.pose.position.y = (j - half + 0.5) * cell
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0

                # ---- 色付け ----
                if unknown:
                    # 未観測は白
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.color.a = 0.5
                else:
                    # 粗いほど赤（std:0→緑, std:10→赤）
                    t = std / 10.0          # 0..1
                    t = max(0.0, min(1.0, t))
                    marker.color.r = 0.0
                    marker.color.g = 1.0 - t
                    marker.color.b = 0.0
                    marker.color.a = 0.6
                    self.get_logger().info(f"({gi},{gj})の粗さ{std}")

                marker.lifetime = Duration(sec=0, nanosec=0)
                msg.markers.append(marker)

        # ---------- 境界線（LINE_LIST） ----------
        lines = Marker()
        lines.header.frame_id = frame
        lines.header.stamp = now
        lines.ns = "grid_lines"
        lines.id = 999999
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0

        lines.scale.x = 0.01
        lines.color.r = 1.0
        lines.color.g = 1.0
        lines.color.b = 1.0
        lines.color.a = 0.5

        z = 0.011

        # ローカル座標でグリッド外枠を作る
        x0 = - (N * cell) / 2.0
        y0 = - (N * cell) / 2.0
        width  = N * cell
        height = N * cell

        for ii in range(N + 1):
            x = x0 + ii * cell
            lines.points.append(Point(x=x, y=y0,        z=z))
            lines.points.append(Point(x=x, y=y0+height, z=z))

        for jj in range(N + 1):
            y = y0 + jj * cell
            lines.points.append(Point(x=x0,       y=y, z=z))
            lines.points.append(Point(x=x0+width, y=y, z=z))

        msg.markers.append(lines)

        self.msg = msg
        self.grid_pub.publish(msg)
        self.get_logger().info("grid書き出し完了")





def main(args=None):
    rclpy.init(args=args)
    node = Cost_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

