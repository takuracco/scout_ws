cmake_minimum_required(VERSION 3.8)
project(roughness_cul)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/roughness_cul_Node/roughness_cul.cpp
  include/robot_node/robot.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Roughness_cul_Node"
  EXECUTABLE roughness_cul_Node
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
