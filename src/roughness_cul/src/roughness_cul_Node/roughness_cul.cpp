#include "roughness_cul.hpp"

roughness_cul::roughness_cul(): rclcpp::Node("roughness_cul"){
    LiDAR_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("LiDAR_Data_out", 10, std::bind(&roughness_cul::pcl_sub_callback, this, std::placeholders::_1));
    Odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("Odom", 10, std::bind(&roughness_cul::odom_sub_callback, this, std::placeholders::_1));
}

void roughness_cul::pcl_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    (void)msg; 
}

void roughness_cul::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    const auto & p = msg->pose.pose.position;
    robot.pose_set(p.x, p.y, p.z);
}

