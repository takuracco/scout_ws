import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from numpy import np
import open3d as o3d

class vector:
    def __init__(self):
        self.x
        self.y
        self.z

class roughness_cul(Node):
    def __init__(self):
        super.__init__('roughness_cul')

        self.sub = self.create_subscription(
            PointCloud2,
            '/scan',
            self.callback,
            10)
        
        # グローバル変数
        self.points = np.zeros((N, 3), dtype=np.float32)
        self.pose = vector()

    
        
    def SetPose(self, x, y, z):
        self.pose.x = x
        self.pose.y = y
        self.pose.z = z
    
    def callback(self,scan):
        pcl = o3d.io.read_point_cloud(scan, field_names=("x", "y", "z"), skip_nans=True)
        points = np.asarray(pcl.points).copy()
        self.points[:,0] = points[:,0] + self.pose.x
        self.points[:,1] = points[:,1] + self.pose.y
        self.points[:,2] = points[:,2] + self.pose.z

        
    
    


def main(args = None):
    rclpy.init(args=args)
    node = roughness_cul()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
