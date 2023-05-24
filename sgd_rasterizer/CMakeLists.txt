cmake_minimum_required(VERSION 3.5)
project(sgd_rasterizer CXX)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
# )
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(plog)

# find dependencies
find_package(ament_cmake REQUIRED)

## build library
set(library_name ${PROJECT_NAME}_core)
set(CPP_FILES
    src/circle_rasterizer.cpp
    src/ray_rasterizer.cpp)

add_library(${library_name} SHARED ${CPP_FILES})
include_directories(include)

install(
  TARGETS ${library_name}
  DESTINATION lib)

# ament_target_dependencies(${library_name}
    # ${dependencies}
  # )

target_link_libraries(${library_name} plog::plog)
# ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${dependencies})

install(
  DIRECTORY include/
  DESTINATION include/)

install(
  TARGETS ${library_name}
  # EXPORT ${library_name}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  # INCLUDES DESTINATION include
)

### testing
# enable_testing()

# add_executable(test_geotools test/test_geotools.cpp)
# add_executable(test_angle_utils test/test_angle_utils.cpp)
# target_link_libraries(test_geotools gtest_main ${library_name})
# target_link_libraries(test_angle_utils gtest_main)

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

# include(GoogleTest)
# gtest_discover_tests(test_geotools)
# gtest_discover_tests(test_angle_utils)

ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()