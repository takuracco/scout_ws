import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point


from roughness_cost import roughness_cost
from slope_cul import slope_cul

class Cost_Node(Node):
    def __init__(self):
        super().__init__('cost_cul')

        # パラメータ（必要なら declare_parameter で外部指定可）
        self.grid_size = 9          # 9 x 9
        self.cell_res  = 1.0         # 1 m / cell
        self.radius_m  = (self.grid_size // 2) * self.cell_res  # 10 m

        # 初期化
        self.points = np.empty((0, 3), dtype=np.float32)
        self.slope_cost = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        # self.grid_var = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)

        # ロボット現在地（本当はTFやOdomで更新するはず。ここでは固定 or 別APIから更新）
        # self.pose = Pose3D(0., 0., 0.)

        # class定義
        self.roughness = roughness_cost(self.grid_size, self.cell_res, self.get_logger())
        self.slopecost = slope_cul(self.grid_size, self.cell_res, self.get_logger())

        # Publisher / Subscriber
        self.sub = self.create_subscription(PointCloud2, '/scan', self.callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/roughness', 10)
        self.grid_pub = self.create_publisher(MarkerArray, "/grid", 10)

        # timer
        self.Timer = self.create_timer(100, self.timer_callback)

        self.get_logger().info('cost_cul node is up.')


    def callback(self,scan):
        """LiDARのデータの"""
        self.get_logger().info('scanデータ受信')
        points = self.cloud2_to_xyz(scan)

        if points.size == 0:
            self.get_logger().warn("ポイントが 0 件")
            return

        self.points = points
        self.get_logger().info(f"変換後の点群 shape={points.shape}")
        # self.get_logger().info(f"サンプル: {points[:500]}")
        self.roughness.roughness_cul(self.points)
        self.slopecost.set_grid_plane(self.roughness.grid_plane)
        self.slope_cost = self.slopecost.all_slope_cul()
        self.get_logger().info(f"slope_cost:{self.slope_cost}")
        self.publish_roughness()
        self.publish_grid()

    def cloud2_to_xyz(self,msg: PointCloud2) -> np.ndarray:
        """
        PointCloud2 -> (N,3) float32 の np.ndarray に安全変換
        x,y,z 以外は捨てる。NaN はスキップ。
        """
        try:
            # まずは numpy 版で試す（速い）
            arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            if isinstance(arr, np.ndarray):
                # 構造化配列か？
                if isinstance(arr, np.ndarray) and arr.dtype.fields is not None:
                    return np.stack([
                        arr['x'].astype(np.float32, copy=False),
                        arr['y'].astype(np.float32, copy=False),
                        arr['z'].astype(np.float32, copy=False)
                    ], axis=-1)
                else:
                    # すでに (N,3) ならそのまま
                    if arr.ndim == 2 and arr.shape[1] >= 3:
                        return arr[:, :3].astype(np.float32, copy=False)
                    # 形が想定外ならフォールバックへ
        except Exception:
            pass

        # フォールバック：ジェネレータ経由（遅いが確実）
        pts = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32
        )
        # 形を (N,3) にそろえる
        if pts.ndim == 1:
            pts = pts.reshape(-1, 3)
        elif pts.ndim == 2 and pts.shape[1] > 3:
            pts = pts[:, :3]
        return pts
    



    def publish_roughness(self):
        """計算済み self.grid_var を /roughness に配信"""
        self.grid_var = self.roughness.grid_var
        if not hasattr(self, "grid_var"):
            self.get_logger().warn("grid_var が存在しません。")
            return

        msg = Float32MultiArray()
        msg.data = self.grid_var.flatten().tolist()

        msg.layout.dim.append(MultiArrayDimension(label="rows", size=self.grid_size, stride=self.grid_size * self.grid_size))
        msg.layout.dim.append(MultiArrayDimension(label="cols", size=self.grid_size, stride=self.grid_size))
        msg.layout.data_offset = 0

        self.pub.publish(msg)
        self.get_logger().info("粗さマップを Publish しました")


    def publish_grid(self):
        """gridに出力"""
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        cell = self.cell_res
        frame = "base_link"

        marker_id = 0
        

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                non_deta = 0
                
                data = np.sqrt(self.grid_var[i][j])
                if data > 10 :
                    data = 10
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
                marker.pose.position.x = i - self.grid_size / 2 + 0.5
                marker.pose.position.y = j - self.grid_size / 2 + 0.5
                marker.pose.position.z = 0.0

                # 粗さ値 → 色（緑の強さ）に変換
                val = self.grid_var[i][j]
                if val is None or np.isnan(val):
                    val = 0.0
                    non_deta = 1
                else:
                    non_data = 0
                std = float(np.sqrt(max(val, 0.0)))  # 分散→標準偏差
                std = min(std, 10.0)                 # 上限クリップ
                intensity = 1 - std               # 0.0〜1.0 に正規化

                marker.color.r = 0.0
                marker.color.g = intensity
                marker.color.b = 0.0
                marker.color.a = 0.6

                if non_deta :
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.color.a = 0.6

                marker.lifetime = Duration(sec=0, nanosec=0)
                

                msg.markers.append(marker)

        # -------------ここからグリッドの境目表示----------------
        lines = Marker()
        lines.header.frame_id = "base_link"              # いま使ってる frame_id に合わせる
        lines.header.stamp = now
        lines.ns = "grid_lines"
        lines.id = 999999                          # 他とかぶらない固定ID                lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0

        lines.scale.x = 0.02                       # 線の太さ（m）
        lines.color.r = 1.0                        # 白の半透明とか見やすい
        lines.color.g = 1.0
        lines.color.b = 1.0
        lines.color.a = 0.6

        # Z-fighting防止に少し浮かせる
        z = 0.01

        N = self.grid_size                         # 例: 5
        res = self.cell_res                        # 例: 1.0
        # グリッド全体の左下原点（境界線用。中心原点±N/2セル分）
        x0 = - (N * res) / 2.0
        y0 = - (N * res) / 2.0
        width  = N * res
        height = N * res

        # 垂直線 i = 0..N
        for i in range(N + 1):
            x = x0 + i * res
            p1 = Point(x=x, y=y0,       z=z)
            p2 = Point(x=x, y=y0+height,z=z)
            lines.points.append(p1)
            lines.points.append(p2)

        # 水平線 j = 0..N
        for j in range(N + 1):
            y = y0 + j * res
            p1 = Point(x = x0,         y = y, z = z)
            p2 = Point(x = x0 + width, y = y, z = z)
            lines.points.append(p1)
            lines.points.append(p2)

        # …ここにセルCUBEを詰める処理…
        msg.markers.append(lines)

        self.msg = msg
        self.grid_pub.publish(msg)
        self.get_logger().info("grid書き出し完了")

    def timer_callback(self):
        self.grid_pub.publish(self.msg)



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