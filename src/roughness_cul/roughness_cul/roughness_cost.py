import numpy as np
# import pyransac3d as pyrsc

class Pose3D:
    def __init__(self, x=0., y=0., z=0.):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class roughness_cost():
    def __init__(self, grid_size, cell_res, logger):

        # パラメータ（必要なら declare_parameter で外部指定可）
        self.grid_size = grid_size          # 21 x 21
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
        self.grid_culculate(points)
        self.Plane_culculate_PCA()


    def grid_culculate(self, points):
        """自身の座標から見たグリッドに分ける"""

        local_x_min = - (self.grid_size / 2) * self.cell_res  # -10
        local_y_min = - (self.grid_size / 2) * self.cell_res  # -10

        x = points[:, 0]
        y = points[:, 1]

        # 各点がどのセルに入るかのデータ配列
        ix = np.floor((x - local_x_min) / self.cell_res).astype(np.int32)
        iy = np.floor((y - local_y_min) / self.cell_res).astype(np.int32)

        mask = (ix >= 0) & (ix < self.grid_size) & (iy >= 0) & (iy < self.grid_size)
        ix = ix[mask]
        iy = iy[mask]
        pts_in = points[mask]

        # まず「リストの二重配列」を作る
        # self.grid_points = None
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


    # def Plane_culculate(self):
    #     """平面推定プログラム"""
    #     self.grid_plane = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]
    #     self.get_logger().info("平面推定開始")

    #     MAX_POINTS = 500

    #     for i in range(self.grid_size):
    #         for j in range(self.grid_size):
    #             pts = self.grid_points[i][j]

    #             if pts.shape[0] < 3:
    #                 continue

    #             # サンプリング
    #             if pts.shape[0] > MAX_POINTS:
    #                 idx = np.random.choice(pts.shape[0], MAX_POINTS, replace=False)
    #                 pts_use = pts[idx]
    #             else:
    #                 pts_use = pts

    #             try:
    #                 plane = pyrsc.Plane()

    #                 equation, inliers = plane.fit(pts_use, 0.01)  
    #                 self.grid_plane[i][j] = equation

    #             except Exception as e:
    #                 self.get_logger().error(f"({i},{j}) 平面推定エラー: {e}")

    #     self.get_logger().info("平面推定完了")

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
        self.grid_plane = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        self.grid_var = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)
        self.get_logger().info("平面推定(PCA)開始")

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                pts = self.grid_points[i][j]  # shape=(N,3)

                if pts.shape[0] < 3:
                    # 点が少なすぎるセルはスキップ or 前の値を流用とか
                    continue

                # 点が多すぎる場合だけ、軽くサンプリング（先頭だけ使う例）
                MAX_POINTS = 500
                if pts.shape[0] > MAX_POINTS:
                    pts_use = pts[:MAX_POINTS]
                else:
                    pts_use = pts

                try:
                    plane_eq, roughness = self.fit_plane_pca(pts_use)

                    # 平面の係数を保存
                    self.grid_plane[i][j] = plane_eq
                    self.grid_var[i][j] = roughness

                    # 粗さマップ用の配列があるならそこに保存
                    # 例: self.grid_roughness[i][j] = roughness
                    # self.grid_roughness[i][j] = roughness

                except Exception as e:
                    self.get_logger().error(f"({i},{j}) 平面推定(PCA)エラー: {e}")

        self.get_logger().info("平面推定(PCA)完了")
        self.get_logger().info("粗さ推定完了")


    # def Plane_culculate(self):
    #     """平面推定プログラム"""
    #     plane = pyrsc.Plane()
    #     self.get_logger().info("平面推定開始")
    #     MAX_POINTS = 500

    #     for i in range(self.grid_size):
    #         for j in range(self.grid_size):
    #             pts = self.grid_points[i][j]

    #             if pts.shape[0] < 3:
    #                 # self.get_logger().warn(f"({i},{j}) セルに十分な点がありません")
    #                 continue
    #             if pts.shape[0] > MAX_POINTS:
    #                 idx = np.random.choice(pts.shape[0], MAX_POINTS, replace=False)
    #                 pts_use = pts[idx]
    #             else:
    #                 pts_use = pts

    #             try:
    #                 equation, inliers = plane.fit(pts_use, 0.01)
    #                 self.grid_plane[i][j] = equation
    #             except Exception as e:
    #                 self.get_logger().error(f"({i},{j}) 平面推定エラー: {e}")
    #             # try:
    #             #     equation, inliers = plane.fit(pts, 0.01)
    #             #     self.grid_plane[i][j] = equation
    #             #     # self.get_logger().info(f"({i},{j}) 平面推定完了")
    #             # except Exception as e:
    #             #     self.get_logger().error(f"({i},{j}) 平面推定エラー: {e}")
    #     self.get_logger().info("平面推定完了")

    # def convert_plane_high(self, x, y, i, j):
    #     """セル(i,j)の平面から高さ(z)を求める"""
    #     try:
    #         a, b, c, d = self.grid_plane[i][j]
    #     except Exception:
    #         return np.nan  # 平面が存在しない場合

    #     if abs(c) < 1e-6:
    #         # z方向成分が小さすぎると不安定なのでスキップ
    #         return np.nan

    #     return -(a * x + b * y + d) / c

    # def roughness_culculate(self):
    #     """分散から粗さを計測する"""
        

    #     for i in range (self.grid_size):
    #         for j in range(self.grid_size):

    #             pts = self.grid_points[i][j]
    #             eq = self.grid_plane[i][j]
    #             if eq is not None and pts.shape[0] > 3:
    #                 z_est = self.convert_plane_high(pts[:, 0], pts[:, 1], i, j)
    #                 residual = pts[:, 2] - z_est
    #                 roughness = np.mean(residual ** 2)
                    
        

    

    

        
                
        


