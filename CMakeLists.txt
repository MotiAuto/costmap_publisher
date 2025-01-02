cmake_minimum_required(VERSION 3.5)
project(costmap_publisher)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(target_costmap_publisher SHARED
  DIRECTORY src
)
target_include_directories(target_costmap_publisher
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_costmap_publisher
  PLUGIN "costmap_publisher::CostMapPublisher"
  EXECUTABLE costmap_publisher_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)