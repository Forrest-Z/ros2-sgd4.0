cmake_minimum_required(VERSION 3.5)
project(sgd_controller)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(plog)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sgd_util REQUIRED)

include_directories(include)

# add executable
add_executable(subsum_controller src/subsum_controller.cpp)
target_link_libraries(subsum_controller plog::plog)
ament_target_dependencies(subsum_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sgd_msgs
)

add_executable(master_control_unit src/master_control_unit.cpp)
ament_target_dependencies(master_control_unit
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  sgd_msgs
)

install(
  TARGETS master_control_unit subsum_controller
  DESTINATION lib/${PROJECT_NAME})

add_library(sgd_progress_checker SHARED
  plugins/sgd_progress_checker.cpp)

ament_target_dependencies(sgd_progress_checker
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sgd_msgs
  nav2_core
  nav_msgs
)

# prevent pluginlib from using boost
target_compile_definitions(sgd_progress_checker PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_library(sgd_local_planner SHARED
  plugins/sgd_local_planner.cpp
)

ament_target_dependencies(sgd_local_planner
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  sgd_msgs
)

# prevent pluginlib from using boost
target_compile_definitions(sgd_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  TARGETS sgd_progress_checker sgd_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(
  DIRECTORY include/
  DESTINATION include/)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(sgd_local_planner sgd_progress_checker)
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
#pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
ament_package()
