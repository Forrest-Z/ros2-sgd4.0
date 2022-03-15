cmake_minimum_required(VERSION 3.5)
project(multilateration_lib)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

#include(FetchContent)
#FetchContent_Declare(
#  googletest
#  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
#)
#FetchContent_MakeAvailable(googletest)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sgd_util REQUIRED)

include_directories(include)

set(library_name multilateration_core)
add_library(${library_name} STATIC
  src/levmarq.cpp
)
ament_target_dependencies(${library_name}
    Eigen3
    eigen3_cmake_module
    sgd_util
)
target_link_libraries(${library_name} sgd_util::sgd_util_core)

#target_include_directories(${library_name} PUBLIC
#      ${Eigen3_INCLUDE_DIRS})
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_export_targets(${library_name})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

### testing
#enable_testing()

#add_executable(unit_test_nmea_parser test/unit_test_nmea_parser.cpp)
#target_link_libraries(unit_test_nmea_parser gtest_main gps_parser_core)

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

#include(GoogleTest)
#gtest_discover_tests(unit_test_nmea_parser)