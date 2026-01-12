#include "RoughnessSlopeAstar/RoughnessSlopeAstar.hpp"

RoughnessSlopeAstarNode::RoughnessSlopeAstarNode(): Node("roughness_slope_astar_node"),robot(robot_t), astar_plan(W, H, center) {
    //parameter
    State = RobotState::Start;
    
    // publisher
    cmd_pub_       = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_controller/cmd_vel_unstamped",rclcpp::QoS(10));

    //subscription
    odom_sub_      = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&RoughnessSlopeAstarNode::on_odom, this, _1));
    roughness_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("roughness", 1, std::bind(&RoughnessSlopeAstarNode::on_roughness_cost, this, _1));
    slope_sub_     = this->create_subscription<std_msgs::msg::Float32MultiArray>("slope", 1, std::bind(&RoughnessSlopeAstarNode::on_slope_cost, this, _1));

    plan_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(astar_t)), std::bind(&RoughnessSlopeAstarNode::plan_tick, this));
    ctrl_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(robot_t)), std::bind(&RoughnessSlopeAstarNode::control_tick, this));
}

// callback処理
void RoughnessSlopeAstarNode::on_odom(nav_msgs::msg::Odometry::SharedPtr msg){
    astar_plan.set_odom(msg);
    robot.set_odom(msg);
}

void RoughnessSlopeAstarNode::on_roughness_cost(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    astar_plan.set_roughness_cost(msg);
}

void RoughnessSlopeAstarNode::on_slope_cost(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    astar_plan.set_slope_cost(msg);
}

//Timer処理
void RoughnessSlopeAstarNode::plan_tick(){
    astar_plan.Astar_Plan();
    if(astar_plan.get_no_path()){
        RCLCPP_INFO(this->get_logger(), "no path error");
        return;
    }
    path_ = astar_plan.get_path();
    path_i = 1;
    get_path = (path_.size() >= 2);
}

void RoughnessSlopeAstarNode::control_tick(){
    switch(State){
        case RobotState::Start:
            if(get_path){
                State = RobotState::Moving;
            }
            break;

        case RobotState::Moving:
            if(path_.size() < 2){
                State = RobotState::Goal;
                vel = robot.get_vel_cmd();
                cmd_vel.linear.x = vel.x;
                cmd_vel.angular.z = vel.yaw;
                cmd_pub_->publish(cmd_vel);
                RCLCPP_INFO(this->get_logger(), "ゴールにたどり着いた");
                break;
            }
            robot.Moving_Robot(path_[path_i]);
            if(robot.is_finished()){
                path_i++;
            }
            vel = robot.get_vel_cmd();
            cmd_vel.linear.x = vel.x;
            cmd_vel.angular.z = vel.yaw;
            cmd_pub_->publish(cmd_vel);
            break;
            
        case RobotState::Goal:
            cmd_vel.linear.x = 0.0f;
            cmd_vel.angular.z = 0.0f;
            cmd_pub_->publish(cmd_vel);
            break;    
    }
}

//helper
// void RoughnessSlopeAstarNode::publish_stop(){

// }
