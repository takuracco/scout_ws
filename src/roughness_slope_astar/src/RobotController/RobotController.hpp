/*-------------------------------------------------------------------------
ロボットを動かす計算をするクラス

入力

出力
/cmd_vel
-------------------------------------------------------------------------*/
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


        float yaw_err_max = 3;    //角度誤差(度数法)
        float W_MAX = 1.2f;       //角速度最大値(rad/s)
        float vel_err_max = 0.01f;//距離の誤差(m)
        float D_MAX = 5.0f;       //最大速度(m/s)

        bool path_get = false;

    public:
        RobotController(float _t);

        void set_target_cell(Cell target);

        void set_odom(nav_msgs::msg::Odometry msg);

        void setup();

        void Moving_Robot();

        
        Pose2D get_vel_cmd();
        
    private:
        float normalize_angle(float angle);
        float get_yaw_err();
        bool Turn_Robot();
        bool Straight_Robot();
        void Stop_Robot();
};