cmake_minimum_required(VERSION 3.5)
project(sgd_local_costmap)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(plog)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

set(executable_name obstacle_detector)
set(CPP_FILES src/obstacle_detector.cpp src/obstacle.cpp)
set(HPP_FILES
    include/sgd_local_costmap/point.hpp
)

add_executable(${executable_name} ${CPP_FILES} ${HPP_FILES})
target_link_libraries(${executable_name} plog::plog)
ament_target_dependencies(${executable_name}
    rclcpp
    rclcpp_lifecycle
    visualization_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    sgd_msgs
    geometry_msgs
)

install(TARGETS
  ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
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
