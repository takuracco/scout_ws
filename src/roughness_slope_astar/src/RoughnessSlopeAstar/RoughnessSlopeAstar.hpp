#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <vector>
#include <cmath>

#include "AstarPlanner/AstarPlanner.hpp"
#include "RobotController/RobotController.hpp"
#include "Library.hpp"


class RoughnessSlopeAstarNode : public rclcpp::Node {
    private:
        int W = 50;
        int H = 50;
        Cell center = {1, 1};

        //ROS
        //subscription
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr roughness_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr slope_sub_;
        //publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        //timer
        rclcpp::TimerBase::SharedPtr plan_timer_;
        rclcpp::TimerBase::SharedPtr ctrl_timer_;
        //timerの制御周期(s)
        float robot_t = 0.1;
        float astar_t = 1;


        std::vector<Cell> path_;

        //インスタンス
        RobotController robot;
        AstarPlanner astar_plan;
        

    public:
        RoughnessSlopeAstarNode();
    
    
    
    //各クラスのインスタンス
    
    private:
        //callback
        void on_odom(const nav_msgs::msg::Odometry & msg);
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