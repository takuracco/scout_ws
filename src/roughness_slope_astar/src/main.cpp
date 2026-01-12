#include <rclcpp/rclcpp.hpp>
#include "RoughnessSlopeAstar/RoughnessSlopeAstar.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoughnessSlopeAstarNode>());
  rclcpp::shutdown();
  return 0;
}