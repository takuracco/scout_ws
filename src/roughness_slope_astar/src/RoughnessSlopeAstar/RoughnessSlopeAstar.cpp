#include "RoughnessSlopeAstar/RoughnessSlopeAstar.hpp"

RoughnessSlopeAstarNode::RoughnessSlopeAstarNode(): Node("roughness_slope_astar_node"),robot(robot_t), astar_plan(W, H, center) {
    //parameter
    State = RobotState::Start;
    astar_plan.set_cost_mode(cost_mode);
    astar_plan.set_goal_cell(goal);
    robot.setup();
    
    // publisher
    cmd_pub_       = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_controller/cmd_vel_unstamped",rclcpp::QoS(10));

    //subscription
    odom_sub_      = this->create_subscription<nav_msgs::msg::Odometry>("diff_drive_controller/odom", 1, std::bind(&RoughnessSlopeAstarNode::on_odom, this, _1));
    roughness_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("roughness_cost", 1, std::bind(&RoughnessSlopeAstarNode::on_roughness_cost, this, _1));
    slope_sub_     = this->create_subscription<std_msgs::msg::Float32MultiArray>("slope_cost", 1, std::bind(&RoughnessSlopeAstarNode::on_slope_cost, this, _1));

    plan_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(astar_t)), std::bind(&RoughnessSlopeAstarNode::plan_tick, this));
    ctrl_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(robot_t)), std::bind(&RoughnessSlopeAstarNode::control_tick, this));

    RCLCPP_INFO(this->get_logger(), "起動");
}

// callback処理
void RoughnessSlopeAstarNode::on_odom(nav_msgs::msg::Odometry::SharedPtr msg){
    get_odom(msg);
    astar_plan.set_odom(odom);
    robot.set_odom(odom);
    // RCLCPP_INFO(this->get_logger(),"自己位置更新");
}

void RoughnessSlopeAstarNode::on_roughness_cost(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    astar_plan.set_roughness_cost(msg);
    if(!get_roughness)get_roughness = true;
    // RCLCPP_INFO(this->get_logger(),"get_roughness_cost");
}

void RoughnessSlopeAstarNode::on_slope_cost(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    astar_plan.set_slope_cost(msg);
    if(!get_slope)get_slope = true;
    // RCLCPP_INFO(this->get_logger(),"get_slope_cost");
}

//Timer処理
void RoughnessSlopeAstarNode::plan_tick(){
    switch(cost_mode){
        case CostMode::DIST_ONLY:
            break;
        case CostMode::ROUGHNESS_ONLY:
            if(!get_roughness){
                return;
            }
            break;
        case CostMode::ROUGHNESS_SLOPE:
            // RCLCPP_INFO(this->get_logger(),"planの更新");
            if(!get_roughness || !get_slope){
                // RCLCPP_INFO(this->get_logger(),"受信しきれてないよ");
                return;
            }
            break;
    }
    astar_plan.Astar_Plan();
    if(astar_plan.get_no_path()){
        RCLCPP_INFO(this->get_logger(), "no path error");
        return;
    }
    path_ = astar_plan.get_path();
    path_i = 1;
    get_path = (path_.size() >= 2);
    // RCLCPP_INFO(this->get_logger(),"パスの作成を実行してるよ%ld",path_.size());
}

void RoughnessSlopeAstarNode::control_tick(){
    switch(State){
        case RobotState::Start:
            RCLCPP_INFO(this->get_logger(),"経路生成の受信待ち");
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
            // RCLCPP_INFO(this->get_logger(), "移動中(%d,%d)　速度(%f,%f,%f)", path_[path_i].x, path_[path_i].y, vel.x, vel.y, vel.yaw);
            // RCLCPP_INFO(this->get_logger(), "odom:%f, %f, %f", Od.x, Od.y, Od.yaw);
            // RCLCPP_INFO(this->get_logger(), "state:%d", robot.get_move_state());
            printf("%f ,%f ,%f ,%f ,%f ,%f ,%d ,%d \n", vel.x, vel.y, vel.yaw, odom.x, odom.y, odom.yaw, path_[path_i].x, path_[path_i].y);
            break;
            
        case RobotState::Goal:
            cmd_vel.linear.x = 0.0f;
            cmd_vel.angular.z = 0.0f;
            cmd_pub_->publish(cmd_vel);
            break;    
    }
}


void RoughnessSlopeAstarNode::get_odom(nav_msgs::msg::Odometry::SharedPtr msg){
    odom.x = msg->pose.pose.position.x + start_x;
    odom.y = msg->pose.pose.position.y + start_y;
    odom.z = msg->pose.pose.position.z + start_z;
    float x = msg->pose.pose.orientation.x;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;

    odom.yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    odom.roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    odom.pitch = std::asin(2.0 * (w * y - z * x));
}
//helper
// void RoughnessSlopeAstarNode::publish_stop(){

// }
