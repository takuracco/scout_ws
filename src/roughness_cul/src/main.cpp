#include "roughness_cul_Node/roughness_cul.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roughness_cul>());
  rclcpp::shutdown();
  return 0;
}