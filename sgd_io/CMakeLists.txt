cmake_minimum_required(VERSION 3.5)
project(sgd_io)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
FetchContent_MakeAvailable(googletest)

find_package(ament_cmake REQUIRED)

## build library
include_directories(include)

set(library_name ${PROJECT_NAME}_core)
set(CPP_FILES src/serial.cpp include/sgd_io/io_exception.hpp)

add_library(${library_name} SHARED
  ${CPP_FILES})

ament_export_targets(${library_name} HAS_LIBRARY_TARGET)

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

add_executable(test_serial test/test_serial.cpp)
target_link_libraries(test_serial gtest_main ${library_name})

#file(COPY test/data DESTINATION ${CMAKE_BINARY_DIR}/lib/test)

include(GoogleTest)
gtest_discover_tests(test_serial)

ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()
