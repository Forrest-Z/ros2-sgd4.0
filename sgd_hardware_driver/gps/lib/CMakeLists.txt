cmake_minimum_required(VERSION 3.5)
project(gps_parser_lib)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(include)

set(library_name gps_parser_core)
add_library(${library_name} STATIC
  src/nmea_parser.cpp
  src/ubx_parser.cpp
  include/nmea_param.hpp
)

ament_target_dependencies(${library_name} INTERFACE
  TinyXML2)
ament_export_dependencies(tinyxml2_vendor TinyXML2)
ament_export_targets(${library_name})

target_include_directories(${library_name} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

### testing
enable_testing()

add_executable(unit_test_nmea_parser test/unit_test_nmea_parser.cpp)
target_link_libraries(unit_test_nmea_parser gtest_main gps_parser_core)

include(GoogleTest)
gtest_discover_tests(unit_test_nmea_parser)