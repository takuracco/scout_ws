import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
from numpy import np
import open3d as o3d
import pyransac3d as pyrsc

class vector:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class roughness_cul(Node):
    def __init__(self):
        super.__init__('roughness_cul')

        self.sub = self.create_subscription(
            PointCloud2,
            '/scan',
            self.callback,
            10)
        
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/roughness',
            10
        )
        
        # グローバル変数
        self.points = []
        self.pose = vector()

    
        
    def SetPose(self, x, y, z):
        self.pose.x = x
        self.pose.y = y
        self.pose.z = z
    
    def callback(self,scan):
        """LiDARのデータの"""
        PCL_points = np.array(list(pc2.read_points(scan, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        self.points[:,0] = PCL_points[:,0] + self.pose.x
        self.points[:,1] = PCL_points[:,1] + self.pose.y
        self.points[:,2] = PCL_points[:,2] + self.pose.z
        self.grid_culculate
        self.Plane_culculate
        self.roughness_culculate
        self.pub(self.grid_var)
        

    def grid_culculate(self):
        """自身の座標から見たグリッドに分ける"""
        ox = -10  #オフセットx 左下のx座標 左下の座標を(0,0)とするために
        oy = -10  #オフセットy
        res = 1   #1セルのサイズ(m)
        size = 21 #セル数

        x = self.points[:, 0]
        y = self.points[:, 1]

        # 各点がどのセルに入るかのデータ配列
        ix = np.floor((x - ox) / res).astype(np.int32)
        iy = np.floor((y - oy) / res).astype(np.int32)

        mask = (ix >=0) & (ix < size) & (iy >=0) & (iy<size)
        ix = ix[mask]
        iy = iy[mask]
        pts_in = self.points[mask]
        
        # self.grid_points = [[[] for _ in range(size)] for _ in range(size)]
        # for k in range(len(ix)):
        #     self.grid_points[ix[k]][iy[k]].append(pts_in[k])
        
        self.grid_points = [[[] for _ in range(size)]for _ in range(size)]
        for i in range(size):
            for j in range(size):
                if self.grid_points[i][j]:
                    self.grid_points[i][j] = np.vstack(self.grid_points[i][j]).astype(np.float32)
                else:
                    self.grid_points[i][j] = np.empty((0,3), dtype = np.float32)


    def Plane_culculate(self):
        """平面推定プログラム"""
        size = 21 #セル数

        plane = pyrsc.Plane()

        self.grid_plane = [[[] for _ in range(size)] for _ in range(size)]

        for i in range(size):
            for j in range(size):
                equation, inliers = plane.fit(self.grid_points[i][j], 0.01)
                self.grid_plane[i][j] = equation

    def convert_plane_high(self,x,y,i,j):
        return -1 * ( self.grid_plane[i][j][1] * x + self.grid_plane[i][j][2] * y + self.grid_plane[i][j][4] ) / self.grid_plane[i][j][3]

    def roughness_culculate(self):
        """分散から粗さを計測する"""
        size = 21 #セル数

        self.grid_var = [[[] for _ in range(size)] for _ in range(size)]

        for i in range (size):
            for j in range(size):
                self.grid_var[i][j] = np.mean((self.grid_points[i][j] - self.convert_plane_high(self.grid_points[i][j][1],self.grid_points[i][j][2],i,j)) ** 2)
                
        
    
    


def main(args = None):
    rclpy.init(args=args)
    node = roughness_cul()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
