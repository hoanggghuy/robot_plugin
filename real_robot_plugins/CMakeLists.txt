cmake_minimum_required(VERSION 3.5)
project(real_robot_plugins)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

add_library(is_battery_low_condition_bt_node SHARED
  src/is_battery_low_condition.cpp
)

target_include_directories(is_battery_low_condition_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(is_battery_low_condition_bt_node
  rclcpp
  sensor_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
)

install(TARGETS is_battery_low_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib  
  RUNTIME DESTINATION bin
)
ament_export_libraries(is_battery_low_condition_bt_node)
target_compile_definitions(is_battery_low_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_package()