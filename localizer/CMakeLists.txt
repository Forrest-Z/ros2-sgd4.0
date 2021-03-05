cmake_minimum_required(VERSION 3.5)
project(localizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(
  include
)

install(TARGETS
  simple_gps_localizer
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME})

# add executable
add_executable(simple_gps_localizer src/simple_gps_localizer.cpp)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  message_filters
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  nav2_util
  nav2_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

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
