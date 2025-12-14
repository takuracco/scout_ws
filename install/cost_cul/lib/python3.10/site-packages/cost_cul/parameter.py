
class Parameter():
    def __init__(self):
        # 表示サイズ
        self.grid_size = 9          # 9 x 9
        self.radius_m  = (self.grid_size // 2) * self.cell_res  # 10 m
        # セルの大きさ
        self.cell_res  = 1.0         # 1 m / cell

        # ロボットの初期位置
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_z     = 0.0
        self.robot_yaw   = 0.0
        self.robot_roll  = 0.0
        self.robot_pitch = 0.0

        # LiDARの位置
        self.LiDAR_x     = 0.0
        self.LiDAR_y     = 0.0
        self.LiDAR_z     = 0.3
        self.LiDAR_roll  = 0.0
        self.LiDAR_yaw   = 0.0
        self.LiDAR_pitch = 0.0
