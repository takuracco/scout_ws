#include "RobotController/RobotController.hpp"

RobotController::RobotController(float _t):TurnPID(_t),StraightPID(_t){
    //インスタンス
    moving_state = MovingState::path_planning;
}


void RobotController::set_odom(Odometry msg){
    //自己位置の取得
    odom = msg;
}

void RobotController::setup(){
    TurnPID.SetParam(2, 0.02, 0.005);
    StraightPID.SetParam(1, 0.02, 0.008);
}

void RobotController::Moving_Robot(Cell target){
    //ロボットを動かす
    switch(moving_state){
        case MovingState::path_planning:
            set_target_cell(target); 
            Stop_Robot();
            if(path_get){
                moving_state = rotation;
            }
            break;
        
        case MovingState::rotation:
            if(Turn_Robot()){
                moving_state = moving;
            }
            break;
        
        case MovingState::moving:
        if(Straight_Robot()){
            moving_state = finish;
        }
        break;
        
        case MovingState::finish:
        path_get = false;
        Stop_Robot();
            break;
    }
}

Pose2D RobotController::get_vel_cmd(){
    return vel_cmd;
}

MovingState RobotController::get_move_state(){
    return moving_state;
}

bool RobotController::is_finished(){
    return moving_state == finish;
}

Odometry RobotController::get_odom(){
    return odom;
}

void RobotController::set_target_cell(Cell target){
    //目標値の設定
    if(!path_get){
        target_cell = target;
        path_get = true;
    }
}

float RobotController::normalize_angle(float angle){
    // [-pi, pi) に正規化
    angle = std::fmod(angle + static_cast<float>(M_PI), static_cast<float>(2.0 * M_PI));
    if (angle < 0.0f) angle += static_cast<float>(2.0 * M_PI);
    return angle - static_cast<float>(M_PI);
}

float RobotController::get_yaw_err(){
    //yaw角の調整用
    float dx = (target_cell.x + 0.5) - odom.x;
    float dy = (target_cell.y + 0.5) - odom.y;
    return normalize_angle(std::atan2(dy, dx) - odom.yaw);
}

float RobotController::get_err(){
    float dx = (target_cell.x + 0.5) - odom.x;
    float dy = (target_cell.y + 0.5) - odom.y;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

bool RobotController::Turn_Robot(){
    vel_cmd.x = 0.0;
    vel_cmd.y = 0.0;

    const float yaw_err = get_yaw_err();

    const float YAW_TOL = yaw_err_max * M_PI / 180.0f;

    if(std::fabs(yaw_err) < YAW_TOL){
        TurnPID.reset();
        vel_cmd.yaw = 0.0;
        return true;
    }

    float w =  TurnPID.culcurate(0.0f, yaw_err);
    vel_cmd.yaw = (std::fabs(w) > W_MAX) ? std::copysign(W_MAX, w) : w;
    
    return false;
}

bool RobotController::Straight_Robot(){
    vel_cmd.y = 0.0f;
    vel_cmd.yaw = 0.0f;

    const float vel_err = get_err();

    if(std::fabs(vel_err) < vel_err_max){
        StraightPID.reset();
        vel_cmd.x = 0.0f;
        return true;
    }

    float d = StraightPID.culcurate(vel_err, 0);
    vel_cmd.x = (std::fabs(d) > D_MAX) ? std::copysign(D_MAX, d) : d;
    return false;
}

void RobotController::Stop_Robot(){
    vel_cmd.x = 0.0f;
    vel_cmd.y = 0.0f;
    vel_cmd.yaw = 0.0f;
}