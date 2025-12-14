import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class grid_management:
    """グリッドの管理用クラス"""
    def __init__(self, cell_res):
        """インスタンス"""
        self.cell_res = cell_res
        self.Cell_point = {}



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

    def get_point(self, msg, robot_x, robot_y, robot_yaw):
        """点群と現在値を受け取って保存する関数"""
        # PCLから配列に変換しつつフィールド座標に
        points = self.cloud2_to_xyz(msg)
        
        # 点群をフィールド座標系に揃える
        px = points[:, 0]
        py = points[:, 1]
        pz = points[:, 2]

        gx = robot_x + px * np.cos(robot_yaw) - py * np.sin(robot_yaw)
        gy = robot_y + px * np.sin(robot_yaw) + py * np.cos(robot_yaw)
        gz = pz

        # ~~グリッドに分ける~~
        # 各点がどのセルに入るかのデータ配列
        for k in range(len(gx)):
            i = int(np.floor(gx[k] / self.cell_res))
            j = int(np.floor(gy[k] / self.cell_res))
            key = (i, j)

            # 格納するセルの作成
            if key not in self.cell:
                self.Cell_point[key] = []

            self.Cell_point[key].append((gx[k], gy[k], gz[k]))
        
    # x,yの引数からグリッド内の点群データを返す関数
    # データ消去(リセット)関数