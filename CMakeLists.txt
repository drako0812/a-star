cmake_minimum_required (VERSION 2.8.11)
project (a-star)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-omit-frame-pointer")

include_directories(source)
add_library(a-star STATIC source/AStar.cpp source/AStar2.cpp util.h)

add_executable(benchmark-astar main.cpp)
target_link_libraries(benchmark-astar a-star benchmark pthread)

