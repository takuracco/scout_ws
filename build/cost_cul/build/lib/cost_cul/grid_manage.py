import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from .parameter import Parameter

class grid_management:
    """グリッドの管理用クラス"""
    def __init__(self, logger):
        """インスタンス"""
        param = Parameter()
        self.cell_res = param.cell_res
        self._logger = logger
        self.Cell_point = {}

        self.grid_plane = {}
        self.grid_var = {} # 粗さ
        self.slope_cost = {} # 方向別コスト

        # パラメータの保存
        self.LiDAR_x     = param.LiDAR_x
        self.LiDAR_y     = param.LiDAR_y
        self.LiDAR_z     = param.LiDAR_z
        self.LiDAR_roll  = param.LiDAR_roll
        self.LiDAR_pitch = param.LiDAR_pitch
        self.LiDAR_yaw   = param.LiDAR_yaw

    def get_logger(self):
        # Node と同じインターフェースにしておく
        return self._logger



    def cloud2_to_xyz(self,msg: PointCloud2) -> np.ndarray:
        """
        PointCloud2 -> (N,3) float32 の np.ndarray に安全変換
        x,y,z 以外は捨てる。NaN はスキップ。
        """
        try:
            # まずは numpy 版で試す
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
                        self.get_logger().info(f"PointCloud2: ndarray already (N,{arr.shape[1]}), N={arr.shape[0]}")
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
    
    def rpy_to_R(self, roll, pitch, yaw):
        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)

        # R = Rz(yaw) * Ry(pitch) * Rx(roll)（ROSでよく使う並び）
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp  ,   cp*sr         ,            cp*cr]
        ], dtype=np.float32)

    def get_point(self, msg, robot_x, robot_y, robot_z, robot_roll, robot_pitch, robot_yaw):
        """点群と現在値を受け取って保存する関数"""
        # PCLから配列に変換
        points = self.cloud2_to_xyz(msg)
        
        t_L = np.array([0.0, 0.0, 0.3], dtype=np.float32)  # URDFのlaser_joint xyz
        R_mount = np.eye(3, dtype=np.float32)              # URDFのlaser_joint rpy（今は0,0,0）

        # LiDAR -> base_link
        pB = (points @ R_mount.T) + t_L  # (N,3)

        # --- 毎回変わる：base_link -> world(odom/map) ---
        R_robot = self.rpy_to_R(robot_roll, robot_pitch, robot_yaw)
        pW = (pB @ R_robot.T) + np.array([robot_x, robot_y, robot_z], dtype=np.float32)

        # 世界座標でセル分け
        gx, gy, gz = pW[:, 0], pW[:, 1], pW[:, 2]

        # ~~グリッドに分ける~~
        # 各点がどのセルに入るかのデータ配列
        for k in range(len(gx)):
            i = int(np.floor(gx[k] / self.cell_res))
            j = int(np.floor(gy[k] / self.cell_res))
            key = (i, j)

            # 格納するセルの作成
            if key not in self.Cell_point:
                self.Cell_point[key] = []
                self.get_logger().info(f"make new cell {key}")

            self.Cell_point[key].append((gx[k], gy[k], gz[k]))
            # self.get_logger().info(f"追加 {gx[k]}, {gy[k]}, {gz[k]}")
       
    # x,yの引数からグリッド内の点群データを返す関数
    # データ消去(リセット)関数