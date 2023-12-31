cmake_minimum_required(VERSION 3.5)
project(sgd_lifecycle_manager)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(include)

set(executable_name lifecycle_manager)
set(dependencies
  rclcpp
  lifecycle_msgs
  tinyxml2_vendor
  TinyXML2
)

# add executable
add_executable(${executable_name} src/lifecycle_manager.cpp src/lf_node_factory.cpp)
ament_target_dependencies(lifecycle_manager ${dependencies})
#target_link_libraries(${executable_name} ${library_name})

add_executable(exit_publisher src/exit_publisher.cpp)
ament_target_dependencies(exit_publisher ${dependencies})

install(
  TARGETS ${executable_name} exit_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/)

ament_package()

### testing
# enable_testing()

# set(test_executable unit_test_lf_node_factory)
# add_executable(${test_executable} test/unit_test_lf_node_factory.cpp src/lf_node_factory.cpp)
# add_test(NAME ${test_executable} COMMAND ${test_executable} ${CMAKE_CURRENT_SOURCE_DIR})
# ament_target_dependencies(${test_executable} ${dependencies})
# target_link_libraries(${test_executable} gtest_main)

# file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/test)

# include(GoogleTest)
# gtest_discover_tests(${test_executable})