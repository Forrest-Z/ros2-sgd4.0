cmake_minimum_required(VERSION 3.5)
project(sgd_util)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)

include_directories(include)

add_library(sgd_util SHARED src/sgd_util.cpp)
ament_target_dependencies(sgd_util
  geometry_msgs
  tf2
  tf2_geometry_msgs)

add_executable(serial src/serial.cpp)
ament_target_dependencies(serial
  rclcpp
  rclcpp_lifecycle
  nav2_util
  sgd_msgs)

  add_executable(serial_mock src/serial_mock.cpp)
  ament_target_dependencies(serial_mock
    rclcpp
    rclcpp_lifecycle
    nav2_util
    sgd_msgs)

target_include_directories(sgd_util
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(
  TARGETS serial serial_mock
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_sgd_util HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs tf2 tf2_geometry_msgs)

install(
  DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY log
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS sgd_util
  EXPORT export_sgd_util
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

#ament_export_include_directories(include)
#ament_export_libraries(sgd_util_lib)
#ament_export_dependencies(rclcpp)
ament_package()
