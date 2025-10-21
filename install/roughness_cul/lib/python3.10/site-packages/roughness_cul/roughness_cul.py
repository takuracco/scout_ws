import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
import pyransac3d as pyrsc

class Pose3D:
    def __init__(self, x=0., y=0., z=0.):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class RoughnessNode(Node):
    def __init__(self):
        super().__init__('roughness_cul')

        # パラメータ（必要なら declare_parameter で外部指定可）
        self.grid_size = 5          # 21 x 21
        self.cell_res  = 1.0         # 1 m / cell
        self.radius_m  = (self.grid_size // 2) * self.cell_res  # 10 m

        # 初期化
        self.points = np.empty((0, 3), dtype=np.float32)
        self.grid_plane = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        self.grid_var = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)

        # ロボット現在地（本当はTFやOdomで更新するはず。ここでは固定 or 別APIから更新）
        # self.pose = Pose3D(0., 0., 0.)

        # Publisher / Subscriber
        self.sub = self.create_subscription(PointCloud2, '/scan', self.callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/roughness', 10)
    
        self.get_logger().info('roughness_cul node is up.')

    # --- 外部から位置更新したい場合に呼ぶ ---
    # def set_pose(self, x, y, z=0.0):
    #     self.pose.x = float(x)
    #     self.pose.y = float(y)
    #     self.pose.z = float(z)
    
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
        self.grid_culculate()
        self.Plane_culculate()
        self.roughness_culculate()
        self.publish_roughness()

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
        

    def grid_culculate(self):
        """自身の座標から見たグリッドに分ける"""

        local_x_min = - (self.grid_size // 2) * self.cell_res  # -10
        local_y_min = - (self.grid_size // 2) * self.cell_res  # -10

        x = self.points[:, 0]
        y = self.points[:, 1]

        # 各点がどのセルに入るかのデータ配列
        ix = np.floor((x - local_x_min) / self.cell_res).astype(np.int32)
        iy = np.floor((y - local_y_min) / self.cell_res).astype(np.int32)

        mask = (ix >= 0) & (ix < self.grid_size) & (iy >= 0) & (iy < self.grid_size)
        ix = ix[mask]
        iy = iy[mask]
        pts_in = self.points[mask]

        # まず「リストの二重配列」を作る
        self.grid_points = [[[] for _ in range(self.grid_size)] for _ in range(self.grid_size)]

        # >>> ここで各セルに点を追加（これが抜けてた）
        for k in range(len(ix)):
            self.grid_points[ix[k]][iy[k]].append(pts_in[k])

        # 各セルを numpy 配列化（空セルは (0,3)）
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if self.grid_points[i][j]:
                    self.grid_points[i][j] = np.vstack(self.grid_points[i][j]).astype(np.float32)
                else:
                    self.grid_points[i][j] = np.empty((0, 3), dtype=np.float32)

        # デバッグ（点が入ってるセル数を数える）
        filled = sum(1 for i in range(self.grid_size) for j in range(self.grid_size)
                    if self.grid_points[i][j].shape[0] > 0)
        self.get_logger().info(f"グリッド分割: 点が入ったセル {filled}/{self.grid_size*self.grid_size}")

        # mask = (ix >=0) & (ix < self.grid_size) & (iy >=0) & (iy<self.grid_size)
        # ix = ix[mask]
        # iy = iy[mask]
        # pts_in = self.points[mask]
        
        # # self.grid_points = [[[] for _ in range(size)] for _ in range(size)]
        # # for k in range(len(ix)):
        # #     self.grid_points[ix[k]][iy[k]].append(pts_in[k])
        
        # self.grid_points = [[[] for _ in range(self.grid_size)]for _ in range(self.grid_size)]
        # for i in range(self.grid_size):
        #     for j in range(self.grid_size):
        #         if self.grid_points[i][j]:
        #             self.grid_points[i][j] = np.vstack(self.grid_points[i][j]).astype(np.float32)
        #         else:
        #             self.grid_points[i][j] = np.empty((0,3), dtype = np.float32)


    def Plane_culculate(self):
        """平面推定プログラム"""
        plane = pyrsc.Plane()

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                pts = self.grid_points[i][j]

                if pts.shape[0] < 3:
                    # self.get_logger().warn(f"({i},{j}) セルに十分な点がありません")
                    continue
                try:
                    equation, inliers = plane.fit(pts, 0.01)
                    self.grid_plane[i][j] = equation
                    # self.get_logger().info(f"({i},{j}) 平面推定完了")
                except Exception as e:
                    self.get_logger().error(f"({i},{j}) 平面推定エラー: {e}")
        self.get_logger().info("平面推定完了")

    def convert_plane_high(self, x, y, i, j):
        """セル(i,j)の平面から高さ(z)を求める"""
        try:
            a, b, c, d = self.grid_plane[i][j]
        except Exception:
            return np.nan  # 平面が存在しない場合

        if abs(c) < 1e-6:
            # z方向成分が小さすぎると不安定なのでスキップ
            return np.nan

        return -(a * x + b * y + d) / c

    def roughness_culculate(self):
        """分散から粗さを計測する"""

        for i in range (self.grid_size):
            for j in range(self.grid_size):

                pts = self.grid_points[i][j]
                eq = self.grid_plane[i][j]
                if eq is not None and pts.shape[0] > 3:
                    z_est = self.convert_plane_high(pts[:, 0], pts[:, 1], i, j)
                    residual = pts[:, 2] - z_est
                    roughness = np.mean(residual ** 2)
                    self.grid_var[i][j] = roughness
        self.get_logger().info("粗さ推定完了")

    def publish_roughness(self):
        """計算済み self.grid_var を /roughness に配信"""
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
                
        

def main(args=None):
    rclpy.init(args=args)
    node = RoughnessNode()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
