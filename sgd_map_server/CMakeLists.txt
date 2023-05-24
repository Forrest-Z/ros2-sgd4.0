cmake_minimum_required(VERSION 3.5)
project(sgd_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

include(FetchContent)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(plog)

find_package(ament_cmake REQUIRED)
#find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sgd_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

find_package(sgd_rasterizer REQUIRED)

include_directories(include)

set(map_server_executable sgd_map_server)

add_executable(${map_server_executable}
  src/map_server.cpp
  src/map_io.cpp
  src/map_io_vector.cpp
  src/map_mode.cpp)
  # src/rasterizer/circle_rasterizer.cpp
  # src/rasterizer/delaunator.cpp
  # src/rasterizer/triangle_rasterizer.cpp)

# set(map_io_library_name map_io)

# set(library_name ${map_server_executable}_core)

# add_library(${map_io_library_name} SHARED
#   src/map_mode.cpp
#   src/map_io.cpp)

# add_library(${library_name} SHARED
#   src/map_server.cpp)

# set(map_io_dependencies
#   yaml_cpp_vendor
#   nav_msgs
#   tf2
#   rclcpp)

set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  tf2
  nav2_msgs
  yaml_cpp_vendor
  std_msgs
  sgd_msgs
  TinyXML2
  sgd_rasterizer)

ament_target_dependencies(${map_server_executable}
  ${map_server_dependencies})

# ament_target_dependencies(${library_name}
#   ${map_server_dependencies})

# ament_target_dependencies(${map_io_library_name}
#   ${map_io_dependencies})

# target_link_libraries(${library_name}
#   ${map_io_library_name})

# target_link_libraries(${map_server_executable}
#   ${library_name})

# target_include_directories(${map_server_executable} SYSTEM PRIVATE
#   ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})

target_link_libraries(${map_server_executable} plog::plog)

# install(TARGETS
#   ${library_name} ${map_io_library_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

install(TARGETS
    ${map_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_export_include_directories(include)
# ament_export_libraries(
#   ${library_name}
#   ${map_io_library_name}
# )
ament_package()
