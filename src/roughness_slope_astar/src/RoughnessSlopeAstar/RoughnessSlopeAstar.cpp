#include "RoughnessSlopeAstar/RoughnessSlopeAstar.hpp"

RoughnessSlopeAstarNode::RoughnessSlopeAstarNode(){
    //parameter
    
}

// callback処理
void RoughnessSlopeAstarNode::on_odom(const nav_msg::msg::Odometry & msg){
    last_odom_ = msg;
}

void RoughnessSlopeAstarNode::on_roughness_cost(const std_msgs::msg::Float32MultiArray & msg){
    last_rough_ = msg;
}

void RoughnessSlopeAstarNode::on_slope_cost(const std_msgs::msg::Float32MultiArray & msg){
    last_slope_ = msg;
}

//Timer処理
void RoughnessSlopeAstarNode::plan_tick(){

}

void RoughnessSlopeAstarNode::control_tick(){

}

//helper
void RoughnessSlopeAstarNode::publish_stop(){

}

//セッターまとめ
void RoughnessSlopeAstarNode::set_odom(){
    
}