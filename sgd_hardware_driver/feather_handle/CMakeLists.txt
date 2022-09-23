cmake_minimum_required(VERSION 3.5)
project(feather_handle)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  json URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz
)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(json plog)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sgd_io REQUIRED)
find_package(sgd_util REQUIRED)

include_directories(include)

set(executable_name feather_handle_ros)
set(dependencies	
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2_geometry_msgs
  geometry_msgs
  sgd_io
  sgd_util
)

# add executable
add_executable(${executable_name} src/feather_handle_ros.cpp include/feather_handle/bno055.hpp include/feather_handle/vl53l1x.hpp)
target_link_libraries(${executable_name} nlohmann_json::nlohmann_json sgd_io::sgd_io_core plog::plog)
ament_target_dependencies(${executable_name} ${dependencies})

install(
  TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
