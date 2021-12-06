cmake_minimum_required(VERSION 3.5)
project(gps)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

#include(FetchContent)
#FetchContent_Declare(
#  googletest
#  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
#)
#FetchContent_MakeAvailable(googletest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)

include_directories(include)
include_directories(lib)

# add executable
add_executable(navilock_ublox6_gps src/navilock_ublox6_gps.cpp)

add_subdirectory(lib)
target_link_libraries(navilock_ublox6_gps gps_parser_core)

ament_target_dependencies(navilock_ublox6_gps
    rclcpp
    nav2_util
    sensor_msgs
    sgd_msgs
)

install(TARGETS navilock_ublox6_gps
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/)

### testing
#enable_testing()

#file(GLOB SRCS src/*.cpp)
#add_executable(test_navilock_ublox6 ${SRCS})

#target_link_libraries(test_navilock_ublox6 gtest_main gps_parser_core)
#ament_target_dependencies(test_navilock_ublox6
#    rclcpp
#    nav2_util
#    sensor_msgs
#    sgd_msgs
#)

#include(GoogleTest)
#gtest_discover_tests(test_navilock_ublox6)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav2_util sensor_msgs sgd_util)

ament_package()