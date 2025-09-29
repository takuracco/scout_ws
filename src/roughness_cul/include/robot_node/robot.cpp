#include "robot_node/robot.hpp"

void Robot::pose_set(float x, float y, float z){
    pose.x = x;
    pose.y = y;
    pose.z = z;
}