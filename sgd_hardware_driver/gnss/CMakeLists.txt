cmake_minimum_required(VERSION 3.5)
project(gnss)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
#FetchContent_Declare(
#  googletest
#  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
#)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(plog)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(sgd_util REQUIRED)
find_package(sgd_io REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(include)

# add executable
add_executable(gnss_node src/gnss_node.cpp src/nmea_parser.cpp src/ntrip_client.cpp)
target_link_libraries(gnss_node sgd_util::sgd_util_core sgd_io::sgd_io_core plog::plog)
ament_target_dependencies(gnss_node
    rclcpp
    rclcpp_lifecycle
    nav2_util
    sensor_msgs
    geometry_msgs
    visualization_msgs
    sgd_msgs
    sgd_util
    sgd_io
    TinyXML2
)

install(TARGETS gnss_node
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
#ament_export_dependencies(rclcpp nav2_util sensor_msgs sgd_util tinyxml2_vendor TinyXML2)

ament_package()
