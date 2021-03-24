cmake_minimum_required(VERSION 3.5)
project(navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(rapidxml REQUIRED)

include_directories(include)

# add executable
add_executable(global_planner src/global_planner.cpp)
ament_target_dependencies(global_planner
  rclcpp
  rclcpp_action
  visualization_msgs
  geometry_msgs
  nav2_msgs
  nav2_util
  sgd_msgs
  rapidxml
)

add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator
  rclcpp
  geometry_msgs
  sensor_msgs
  sgd_msgs
)

install(
  TARGETS global_planner navigator
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
