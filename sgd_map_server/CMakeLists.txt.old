cmake_minimum_required(VERSION 3.5)
project(sgd_map_server)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(plog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
#find_package(sgd_util REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

set(map_server_executable sgd_map_server)

#set(map_saver_cli_executable map_saver_cli)
#set(map_saver_server_executable map_saver_server)

add_executable(${map_server_executable} src/map_server.cpp src/map_io.cpp src/map_mode.cpp)
target_link_libraries(${map_server_executable} plog::plog)


# add_executable(${map_saver_cli_executable}
#   src/map_saver/main_cli.cpp)

# add_executable(${map_saver_server_executable}
#   src/map_saver/main_server.cpp)

# set(library_name ${map_server_executable}_core)

# add_library(${map_io_library_name} SHARED
#   src/map_mode.cpp
#   src/map_io.cpp)

# add_library(${library_name} SHARED
#   src/map_server/map_server.cpp
#   src/map_saver/map_saver.cpp)

# set(map_io_dependencies
#   yaml_cpp_vendor
#   nav_msgs
#   nav2_util
#   tf2)

set(map_server_dependencies
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  std_msgs
  TinyXML2
  yaml_cpp_vendor)

# set(map_saver_dependencies
#   rclcpp
#   nav_msgs
#   nav2_msgs
#   nav2_util)

ament_target_dependencies(${map_server_executable}
  ${map_server_dependencies})

# ament_target_dependencies(${map_saver_cli_executable}
#   ${map_saver_dependencies})

# ament_target_dependencies(${map_saver_server_executable}
#   ${map_saver_dependencies})

# ament_target_dependencies(${library_name}
#   ${map_server_dependencies})

# ament_target_dependencies(${map_io_library_name}
#   ${map_io_dependencies})

install(TARGETS
    ${map_server_executable} #${map_saver_cli_executable} ${map_saver_server_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

ament_export_include_directories(include)
ament_package()
