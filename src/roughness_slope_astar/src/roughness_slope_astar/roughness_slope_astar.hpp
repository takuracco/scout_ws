#include <rclcpp/rclcpp.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class roughness_slope_Astar_node : public rclcpp::Node {
    public:
        RoughSlopeAstarNode() : Node("roughness_slope_astar_node") {
        }
}