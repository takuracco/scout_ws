import numpy as np

from .parameter import Parameter


DIRECTIONS = [
    (1, 0),   # 0: 右
    (1, 1),   # 1: 右上
    (0, 1),   # 2: 上
    (-1, 1),  # 3: 左上
    (-1, 0),  # 4: 左
    (-1, -1), # 5: 左下
    (0, -1),  # 6: 下
    (1, -1)   # 7: 右下
]

class slope_cul():
    def __init__(self, logger, origin_x = 0.0, origin_y = 0.0):
        param = Parameter()
        self.cell_res = param.cell_res
        self._logger = logger
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.max_rad = param.max_rad

        self.grid_plane = {}
        self.slope_cost = {}

    def get_logger(self):
        return self._logger

    def set_grid_plane(self, grid_plane):
        self.grid_plane = grid_plane
        return grid_plane

    def angle_estimation_plane_only(self, x, y):
        """
        平面推定した情報(ax+by+cz+d=0)から、
        セル(x,y)の8方向の傾斜角(theta) [rad] を返す（plane only）
        """
        theta = np.full(8, np.inf, dtype=np.float32)

        plane = self.grid_plane.get((x, y), None)
        if plane is None:
            return np.zeros(8, dtype=np.float32)  # 未推定は0扱い（あなたの方針）

        a, b, c, d = plane
        if abs(c) < 1e-9:
            return theta

        for dir_idx, (dx, dy) in enumerate(DIRECTIONS):
            # 方向ベクトルを正規化（右上など(1,1)対策）
            norm = np.hypot(dx, dy)
            ux, uy = dx / norm, dy / norm

            # 方向微分 dz/ds
            dzds = -(a * ux + b * uy) / c

            # 傾斜角（符号付き）
            theta[dir_idx] = np.float32(np.arctan(dzds))

        return theta


    def slope_cul(self, i, j):
        """
        セル(i,j)に対して、8方向の傾斜角コストを計算する
        return: slope_cost[8] のndarray
        """
        slope_cost = np.full(8, np.inf, dtype=np.float32)
        slope_angle = self.angle_estimation_plane_only(i, j)

        slope_cost = np.clip(np.abs(slope_angle), 0.0, self.max_rad) / self.max_rad # 正規化 (0~1に)
        return slope_cost
    


    def all_slope_cul(self):
        """すべてのグリッドで計算"""
        for (i, j) in self.grid_plane.keys():
            self.slope_cost[(i, j)] = self.slope_cul(i, j)

        return self.slope_cost
    
                
