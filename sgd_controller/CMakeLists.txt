cmake_minimum_required(VERSION 3.5)
project(sgd_controller)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# add executable
add_executable(subsum_controller src/subsum_controller.cpp)
ament_target_dependencies(subsum_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

install(
  TARGETS subsum_controller
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include/)

ament_package()
