#include "rclcpp/rclcpp.hpp"
#include "robot_node/robot.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

class roughness_cul:public rclcpp::Node{
    private:
        Robot robot;
        //publisher

        //subscriber
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LiDAR_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odom_sub;

        //callback
        void pcl_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    public:
        roughness_cul();
};