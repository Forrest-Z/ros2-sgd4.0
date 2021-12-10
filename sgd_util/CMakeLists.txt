cmake_minimum_required(VERSION 3.5)
project(sgd_util)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)
add_library(${library_name} SHARED src/sgd_util.cpp)
ament_target_dependencies(${library_name}
  geometry_msgs
  tf2
  tf2_geometry_msgs)

add_executable(logger src/logger.cpp)
ament_target_dependencies(logger
  rclcpp
  rclcpp_lifecycle
  nav2_util
  sensor_msgs
  nav_msgs)

target_include_directories(${library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(
  TARGETS logger
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_sgd_util HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs tf2 tf2_geometry_msgs)

install(
  DIRECTORY include/
  DESTINATION include/)

install(
  TARGETS ${library_name}
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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()