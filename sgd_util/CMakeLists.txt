cmake_minimum_required(VERSION 3.5)
project(sgd_util)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

## build library
include_directories(include)

set(library_name ${PROJECT_NAME}_core)
set(CPP_FILES src/sgd_util.cpp src/geotools.cpp include/sgd_util/ieee_754_conv.hpp)
set(dependencies
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_library(${library_name} SHARED
  ${CPP_FILES})

ament_target_dependencies(${library_name}
  ${dependencies})

ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

add_executable(visualizer src/visualizer.cpp)
ament_target_dependencies(visualizer
  rclcpp
  geometry_msgs
  visualization_msgs)

add_executable(wgs_to_local src/wgs_to_local.cpp)
ament_target_dependencies(wgs_to_local
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  geometry_msgs)
target_link_libraries(wgs_to_local ${library_name})

install(
  TARGETS visualizer wgs_to_local
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

### testing
enable_testing()

add_executable(test_geotools test/test_geotools.cpp)
target_link_libraries(test_geotools gtest_main ${library_name})

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

include(GoogleTest)
gtest_discover_tests(test_geotools)

ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()