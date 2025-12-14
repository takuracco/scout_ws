import numpy as np
# import pyransac3d as pyrsc

class roughness_cost():
    def __init__(self, cell_res, logger):

        # パラメータ（必要なら declare_parameter で外部指定可）
        self.cell_res  = cell_res         # 1 m / cell
        self.radius_m  = (self.grid_size // 2) * self.cell_res  # 10 m
        # self.grid_var = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)
        self._logger = logger
        # ロボット現在地（本当はTFやOdomで更新するはず。ここでは固定 or 別APIから更新）
        # self.pose = Pose3D(0., 0., 0.)

    def get_logger(self):
        # Node と同じインターフェースにしておく
        return self._logger

    def roughness_cul(self, points):
        """計算"""
        self.grid_points = points
        self.Plane_culculate_PCA()

    def fit_plane_pca(self, pts: np.ndarray):
        """
        pts: shape (N, 3)
        戻り値:
            plane_eq: np.array([a, b, c, d])  (ax + by + cz + d = 0)
            roughness: float
        """
        # 重心
        c = pts.mean(axis=0)  # (3,)
        X = pts - c           # (N,3)

        # 共分散行列 (3x3)
        # Nで割るか(N-1)で割るかは好み。ここではNで割る。
        cov = (X.T @ X) / pts.shape[0]

        # 固有値・固有ベクトル
        # w: 固有値 (3,), v: 列が固有ベクトル (3,3)
        w, v = np.linalg.eigh(cov)  # 対称行列なのでeighでOK (速い)

        # 固有値は昇順に並ぶ: w[0] <= w[1] <= w[2]
        # 最小固有値の固有ベクトルが「平面の法線」
        normal = v[:, 0]
        d = -np.dot(normal, c)

        # 粗さの指標（例）：最小固有値 / 全体
        lam1, lam2, lam3 = w[2], w[1], w[0]  # 大きい順に並べ直すならこう
        roughness = lam3 / (lam1 + lam2 + lam3 + 1e-9)  # 0除算防止

        plane_eq = np.array([normal[0], normal[1], normal[2], d], dtype=np.float32)
        return plane_eq, float(roughness)
    
    def Plane_culculate_PCA(self):
        """平面推定＋粗さ推定（PCA版）"""
        self.grid_plane = {}  # (i,j) -> plane_eq
        self.grid_var = {}    # (i,j) -> roughness
        self.get_logger().info("平面推定(PCA)開始")

        for (i, j), pts in self.grid_points.items():
            pts = self.grid_points[(i,j)]  # shape=(N,3)

            if pts.shape[0] < 3:
                # 点が少なすぎるセルはスキップ or 前の値を流用とか
                continue

            # 点が多すぎる場合だけ、軽くサンプリング（先頭だけ使う例）
            MAX_POINTS = 1000
            if pts.shape[0] > MAX_POINTS:
                pts_use = pts[:MAX_POINTS]
            else:
                pts_use = pts

            try:
                plane_eq, roughness = self.fit_plane_pca(pts_use)

                # 平面の係数を保存
                self.grid_plane[(i, j)] = plane_eq
                self.grid_var[(i, j)] = roughness

            except Exception as e:
                self.get_logger().error(f"({i},{j}) 平面推定(PCA)エラー: {e}")

        self.get_logger().info("平面推定(PCA)完了")
        self.get_logger().info("粗さ推定完了")
