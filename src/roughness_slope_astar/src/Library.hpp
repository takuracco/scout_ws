


struct Pose2D {
  double x{0}, y{0}, yaw{0};
};

struct Odometry{
    float x{0}, y{0}, z{0};
    float roll{0}, pitch{0}, yaw{0};
    //　　x軸回り , y軸回り , z軸回り
}

enum class CostMode : int {
  DIST_ONLY = 0,
  ROUGHNESS_ONLY = 1,
  ROUGHNESS_SLOPE = 2
};

struct Cell{
    int x;
    int y;
};

inline int dir_from_dxdy(int dx, int dy)
{
  // dx,dy は -1,0,1 のどれか（8近傍前提）
  if (dx ==  1 && dy ==  0) return 0;//右
  if (dx ==  1 && dy ==  1) return 1;//右上
  if (dx ==  0 && dy ==  1) return 2;//上
  if (dx == -1 && dy ==  1) return 3;//左上
  if (dx == -1 && dy ==  0) return 4;//左
  if (dx == -1 && dy == -1) return 5;//左下
  if (dx ==  0 && dy == -1) return 6;//下
  if (dx ==  1 && dy == -1) return 7;//右下
  return -1; // error
}

inline int dx_from_dir(int dir)
{
  // dx,dy は -1,0,1 のどれか（8近傍前提）
  if (dir = 0) return 1;
  if (dir = 1) return 1;
  if (dir = 2) return 0;
  if (dir = 3) return -1;
  if (dir = 4) return -1;
  if (dir = 5) return -1;
  if (dir = 6) return 0;
  if (dir = 7) return 1;
  return 0;
}

inline int dy_from_dir(int dir)
{
  // dx,dy は -1,0,1 のどれか（8近傍前提）
  if (dir = 0) return 0;
  if (dir = 1) return 1;
  if (dir = 2) return 1;
  if (dir = 3) return 1;
  if (dir = 4) return 0;
  if (dir = 5) return -1;
  if (dir = 6) return -1;
  if (dir = 7) return -1;
  return 0;
}


