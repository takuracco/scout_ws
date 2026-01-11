#include "RoughnessSlopeAstar/RoughnessSlopeAstar.hpp"

RoughnessSlopeAstarNode::RoughnessSlopeAstarNode(): Node("roughness_slope_astar_node"),robot(robot_t), astar_plan(W, H, center) {
    //parameter
    
    // publisher
    cmd_pub_       = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_controller/cmd_vel_unstamped",rclcpp::QoS(10));

    //subscription
    odom_sub_      = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&RoughnessSlopeAstarNode::on_odom, this, 1));
    roughness_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("roughness", 1, std::bind(&RoughnessSlopeAstarNode::on_roughness_cost, this, 1));
    slope_sub_     = this->create_subscription<std_msgs::msg::Float32MultiArray>("slope", 1, std::bind(&RoughnessSlopeAstarNode::on_slope_cost, this, 1));

    plan_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(astar_t)), std::bind(&RoughnessSlopeAstarNode::plan_tick, this));
    ctrl_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(robot_t)), std::bind(&RoughnessSlopeAstarNode::control_tick, this));
}

// callback処理
void RoughnessSlopeAstarNode::on_odom(const nav_msgs::msg::Odometry & msg){
    astar_plan.set_odom(msg);
    robot.set_odom(msg);
}

void RoughnessSlopeAstarNode::on_roughness_cost(const std_msgs::msg::Float32MultiArray & msg){
    astar_plan.set_roughness_cost(msg);
}

void RoughnessSlopeAstarNode::on_slope_cost(const std_msgs::msg::Float32MultiArray & msg){
    astar_plan.set_slope_cost(msg);
}

//Timer処理
void RoughnessSlopeAstarNode::plan_tick(){
    astar_plan.Astar_Plan();
    path_ = astar_plan.get_path();
}

void RoughnessSlopeAstarNode::control_tick(){

}

//helper
void RoughnessSlopeAstarNode::publish_stop(){

}
