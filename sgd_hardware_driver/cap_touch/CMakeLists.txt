cmake_minimum_required(VERSION 3.5)
project(cap_touch)

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
find_package(sgd_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sgd_io REQUIRED)

include_directories(include)

set(executable_name capacitive_touch)
add_executable(${executable_name} src/capacitive_touch.cpp src/moving_average_filter.cpp)
ament_target_dependencies(${executable_name}
    rclcpp
    rclcpp_lifecycle
    sgd_msgs
    sgd_io
)
target_link_libraries(${executable_name} sgd_io::sgd_io_core)

install(TARGETS
  ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY include/
  DESTINATION include/)

### testing
enable_testing()

add_executable(test_cap_touch test/test_moving_average_filter.cpp src/moving_average_filter.cpp)
target_link_libraries(test_cap_touch gtest_main)

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

include(GoogleTest)
gtest_discover_tests(test_cap_touch)

ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()
