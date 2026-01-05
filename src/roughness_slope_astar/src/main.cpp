#include <rclcpp/rclcpp.hpp>
#include "src/roughness_slope_astar/roughness_slope_astar.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roughness_slope_Astar_node>());
  rclcpp::shutdown();
  return 0;
}