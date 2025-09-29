#include "rclcpp/rclcpp.hpp"
#include "stdio.h"

typedef struct vector {
    float x;
    float y;
    float z;
} vector3;

class Robot{
    private:
        vector3 pose;
    public:
        void pose_set(float x,float y,float z);
        Robot();
};