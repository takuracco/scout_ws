import numpy as np



class slope_cul():
    def __init__(self, grid_size, cell_res):
        self.grid_size = grid_size
        self.cell_res = cell_res
        
        self.grid_var = np.full((self.grid_size, self.grid_size), np.nan, dtype=np.float32)

    def set_grid_var(self, grid_var):
        self.grid_var = grid_var

    def get_cell_center(self, i, j):
        """セル(i,j)の中心を返す関数"""
        a, b, c, d = self.grid_var[i][j]