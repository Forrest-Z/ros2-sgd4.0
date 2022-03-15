cmake_minimum_required(VERSION 3.5)
project(uwb)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sgd_io REQUIRED)
find_package(sgd_util REQUIRED)

include_directories(include)
include_directories(lib)

set(executable_name uwb_node)
add_executable(${executable_name} src/uwb_node.cpp)

add_subdirectory(lib)
target_link_libraries(${executable_name} multilateration_core sgd_io::sgd_io_core)

ament_target_dependencies(${executable_name}
    rclcpp
    rclcpp_lifecycle
    tf2_ros
    geometry_msgs
    sensor_msgs
    visualization_msgs
    sgd_io
    sgd_util
)

install(TARGETS
  ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle geometry_msgs sgd_io)

ament_package()
