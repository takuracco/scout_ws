#include <rclcpp/rclcpp.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <mutex>
#include <optional>
#include <vector>
#include <cmath>

#include "AstarPlanner/AstarPlanner.hpp"
#include "RobotController/RobotController.hpp"
#include "Library.hpp"


class roughness_slope_Astar_node : public rclcpp::Node {
    private:
    //ROS
    //subscription
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr roughness_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr slope_sub_;
    //publisher
    rclcpp::Publisher<Geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    //timer
    rclcpp::TimerBase::SharedPtr plan_timer_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;


    std::optional<nav_msgs::msg::Odometry> last_odom_;
    std::optional<std_msgs::msg::Float32MultiArray> last_rough_;
    std::optional<std_msgs::msg::Float32MultiArray> last_slope_;
    std::vector<Cell> path_;

    public:
    RoughSlopeAstarNode() : Node("roughness_slope_astar_node") {}
    
    
    
    //各クラスのインスタンス
    
    private:
    //callback
    void on_odom(const nav_msg::msg::Odometry & msg);
    void on_roughness_cost(const std_msgs::msg::Float32MultiArray & msg);
    void on_slope_cost(const std_msgs::msg::Float32MultiArray & msg);

    // timers
    void plan_tick();
    void control_tick();

    //helpers
    void publish_stop();
        
    //クラスのセッターまとめ
    void set_odom();

};