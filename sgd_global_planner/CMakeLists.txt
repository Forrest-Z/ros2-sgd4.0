cmake_minimum_required(VERSION 3.5)
project(sgd_global_planner)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# uncomment to add debugging capabilities
# add_compile_options(-g)
# to launch ros2 with gdb options execute the following line
# ros2 run --prefix 'gdb -ex run --args' <pkg> <node> --all-other-launch arguments

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz
)
FetchContent_MakeAvailable(json)

#include(FetchContent)
#FetchContent_Declare(
#  googletest
#  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
#)
#FetchContent_MakeAvailable(googletest)

# find dependencies
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(sgd_util REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
#find_package(nav2_util REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  nav_msgs
  sgd_msgs
  sgd_util
  tf2_ros
  tf2_geometry_msgs
  TinyXML2
  yaml_cpp_vendor
  #nav2_util
)

include_directories(include)

# add executable
set(EXE_NAME global_planner)
add_executable(${EXE_NAME} src/global_planner.cpp src/a_star.cpp src/a_star_node.cpp src/a_star_users.cpp src/path_smoothing.cpp)
target_link_libraries(${EXE_NAME} sgd_util::sgd_util_core nlohmann_json::nlohmann_json)
#target_link_libraries(${EXE_NAME} PRIVATE nlohmann_json::nlohmann_json)
ament_target_dependencies(${EXE_NAME} ${dependencies})

#add_executable(navigator src/navigator.cpp)
#ament_target_dependencies(navigator
#  rclcpp
#  geometry_msgs
#  sensor_msgs
#  sgd_msgs
#  sgd_util
#  tf2_geometry_msgs
#)

#install(
#  TARGETS ${library_name}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION lib)

add_executable(client src/srv_client.cpp)
ament_target_dependencies(client
  rclcpp
  sgd_msgs
)

install(
  TARGETS ${EXE_NAME} client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
  
install(
  DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
#ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

### testing
#enable_testing()

#set(testname unit_test_a_star)
#add_executable(${testname} test/${testname}.cpp)
#target_link_libraries(${testname} gtest_main a_star_core)

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

#include(GoogleTest)
#gtest_discover_tests(${testname})

#ament_package()
