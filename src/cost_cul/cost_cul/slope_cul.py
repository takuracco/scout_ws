import numpy as np

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
    def __init__(self, cell_res, logger, origin_x = 0.0, origin_y = 0.0):
        self.cell_res = cell_res
        self._logger = logger
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.grid_plane = {}
        self.slope_cost = {}

    def get_logger(self):
        return self._logger

    def set_grid_plane(self, grid_plane):
        self.grid_plane = grid_plane

    def get_cell_center(self, i, j):
        """セル(i,j)の中心のz座標を返す関数"""

        plane = self.grid_plane.get((i, j), None)
        if plane is None:
            return np.nan  # 未推定

        a, b, c, d = plane
        if abs(c) < 1e-9:
            return None  # 推定はあるが計算不能

        # 中心の座標
        center_x = self.origin_x + (i + 0.5) * self.cell_res
        center_y = self.origin_y + (j + 0.5) * self.cell_res
        
        if abs(c) < 1e-9:
            return None

        center_z = (- a * center_x - b * center_y - d) / c # 0=ax+by+cz+d

        return center_z

    def slope_cul(self, i, j):
        """
        セル(i,j)に対して、8方向の傾斜角コストを計算する
        return: slope_cost[8] のndarray
        """
        center_z = self.get_cell_center(i, j)
        # 返り値は必ず ndarray にする
        slope_cost = np.full(8, np.inf, dtype=np.float32)

        # セル自体が計算不能なら全部通れない
        if center_z is None:
            return slope_cost

        # 未推定セルは「何もない」扱いにしたいなら 0 を返す（あなたの方針）
        if np.isnan(center_z):
            return np.zeros(8, dtype=np.float32)

        for dir_idx, (dx,dy) in enumerate(DIRECTIONS):
            nx = i + dx
            ny = j + dy

            neighbor_z = self.get_cell_center(nx, ny)

            if np.isnan(neighbor_z):
                slope_cost[dir_idx] = np.inf
                continue
            
            # 高さ
            dz = neighbor_z - center_z
            # 距離(斜めは√2倍)
            horizontal_dist = self.cell_res * (np.sqrt(2) if dx != 0 and dy != 0 else 1)
            # 傾斜角(rad)
            theta = np.arctan(dz / horizontal_dist)

            # 角度がコストになってる(rad) -1.7 ~ 1.7
            slope_cost[dir_idx] = theta

        return slope_cost
    


    def all_slope_cul(self):
        """すべてのグリッドで計算"""
        for (i, j) in self.grid_plane.keys():
            self.slope_cost[(i, j)] = self.slope_cul(i, j)

        return self.slope_cost
    
                
