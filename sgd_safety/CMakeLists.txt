cmake_minimum_required(VERSION 3.5)
project(sgd_safety)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

include_directories(include)

# add executable
set(executable_name ros2_obstacle_checker)
add_executable(${executable_name} src/ros2_obstacle_checker.cpp src/obstacle_checker.cpp)
ament_target_dependencies(${executable_name}
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
)

install(
  TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include/)

### testing
enable_testing()

set(test_name test_obstacle_checker)
add_executable(${test_name} test/test_obstacle_checker.cpp src/obstacle_checker.cpp)
target_link_libraries(${test_name} gtest_main)

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

include(GoogleTest)
gtest_discover_tests(${test_name})

ament_package()
