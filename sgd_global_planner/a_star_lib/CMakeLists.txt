# TODO create library

cmake_minimum_required(VERSION 3.5)
# Default to C++17
set(CMAKE_CXX_STANDARD 17)

set(library_name AStar)

include(FetchContent)
FetchContent_Declare(
  json URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz
)
FetchContent_Declare(
  plog URL https://github.com/SergiusTheBest/plog/archive/refs/tags/1.1.8.tar.gz
)
FetchContent_MakeAvailable(json plog)

find_package(sgd_util REQUIRED)

include_directories(include)
add_library(${library_name}
            src/a_star.cpp
            src/a_star_users.cpp
            src/a_star_path.cpp
            src/a_star_node.cpp)

target_link_libraries(${library_name} plog::plog)
ament_target_dependencies(${library_name} sgd_util)