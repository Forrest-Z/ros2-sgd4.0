cmake_minimum_required(VERSION 3.5)
project(sgd_costmap_plugins)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

set(lib_name ${PROJECT_NAME}_core)

# include(FetchContent)
# FetchContent_Declare(
#   plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
# )
# FetchContent_MakeAvailable(plog)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    sgd_msgs)

add_library(${lib_name} SHARED src/static_vector_layer.cpp)
include_directories(include)

install(TARGETS ${lib_name}
        DESTINATION lib)

pluginlib_export_plugin_description_file(sgd_costmap_plugins static_vector_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()