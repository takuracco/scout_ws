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
    def __init__(self, grid_size, cell_res, logger):
        self.grid_size = grid_size
        self.cell_res = cell_res
        self._logger = logger

        self.grid_plane = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]
        self.slope_cost = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]

    def get_logger(self):
        return self._logger

    def set_grid_plane(self, grid_plane):
        self.grid_plane = grid_plane

    def get_cell_center(self, i, j):
        """セル(i,j)の中心のz座標を返す関数"""
        a, b, c, d = self.grid_plane[i][j]

        center_x = 0
        center_y = 0

        # 中心の座標
        center_x = (i - (self.grid_size / 2 - 0.5)) * self.cell_res
        center_y = (j - (self.grid_size / 2 - 0.5)) * self.cell_res
        
        center_z = (- a * center_x - b * center_y - d) / c # 0=ax+by+cz+d

        return center_z

    def slope_cul(self, i, j):
        """
        セル(i,j)に対して、8方向の傾斜角コストを計算する
        return: slope_cost[8] のndarray
        """
        center_z = self.get_cell_center(i, j)
        slope_cost = np.zeros(8, dtype=np.float32)

        for dir_idx, (dx,dy) in enumerate(DIRECTIONS):
            nx = i + dx
            ny = j + dy

            if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                slope_cost[dir_idx] = np.inf
                continue

            neighbor_z = self.get_cell_center(nx, ny)

            if np.isnan(neighbor_z) or np.isnan(center_z):
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
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                self.slope_cost[i][j] = self.slope_cul(i, j)

        return self.slope_cost
    
                

