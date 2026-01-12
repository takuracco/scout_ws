/*-------------------------------------------------------------------------
ロボットを動かす計算をするクラス

setup()を実行してから動かす

入力
実行周期 秒(float)                 　・・・(インスタンス)
目標のCell番号(Cell)               　・・・Moving_Robot
自己位置(nav_msgs::msg::Odometry)　　・・・set_odom

出力
cmd_vel(Pose2D)　　　　　            ・・・get_vel_cmd

実行部
MovingRobot()
-------------------------------------------------------------------------*/
#ifndef __RobotController_HPP__
#define __RobotController_HPP__

#include <vector>
#include <cmath>
#include <algorithm>
#include <nav_msgs/msg/odometry.hpp>

#include "Library.hpp"
#include "RobotController/PID.hpp"

enum MovingState{
    path_planning,
    rotation,
    moving,
    finish
};

class RobotController{
    private:

        Pose2D vel_cmd;
        Odometry odom;

        PID TurnPID,StraightPID;

        Cell target_cell;

        MovingState moving_state;


        float yaw_err_max = 10;    //角度誤差(度数法)
        float W_MAX = 3.0f;       //角速度最大値(rad/s)
        float vel_err_max = 0.01f;//距離の誤差(m)
        float D_MAX = 5.0f;       //最大速度(m/s)

        bool path_get = false;

    public:
        RobotController(float _t);

        
        void set_odom(Odometry msg);
        
        void setup();
        
        void Moving_Robot(Cell target);
        
        Pose2D get_vel_cmd();

        MovingState get_move_state();

        bool is_finished();
        
        Odometry get_odom();
    private:
        void set_target_cell(Cell target);
        float normalize_angle(float angle);
        float get_yaw_err();
        float get_err();
        bool Turn_Robot();
        bool Straight_Robot();
        void Stop_Robot();
};

#endif