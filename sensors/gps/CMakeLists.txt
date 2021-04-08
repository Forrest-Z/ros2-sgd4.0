cmake_minimum_required(VERSION 3.5)
project(gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sgd_util REQUIRED)
find_package(rapidxml REQUIRED)

include_directories(include)

add_executable(navilock_ublox6_gps src/navilock_ublox6_gps.cpp)
add_executable(gps_transform src/sensor_gps_transform.cpp)

set(library_name gps_sensor_core)
add_library(${library_name} SHARED
  src/nmea_parser.cpp
  include/gps/nmea_param.hpp
)
ament_target_dependencies(${library_name}
    rapidxml
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


#target_include_directories(${library_name} PRIVATE include/gps)

# add executable

ament_target_dependencies(gps_transform
    rclcpp
    sensor_msgs
    sgd_util
)

ament_target_dependencies(navilock_ublox6_gps
    rclcpp
    rapidxml
    sensor_msgs
)
target_link_libraries(navilock_ublox6_gps ${library_name})


install(TARGETS
  gps_transform
  navilock_ublox6_gps
  #nmea_parser
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME})
  
#install(DIRECTORY params
#  DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(include)
#ament_export_libraries(${library_name} rapidxml)
ament_export_dependencies(rclcpp sensor_msgs sgd_util std_msgs rapidxml)

ament_package()
