cmake_minimum_required (VERSION 2.8.11)
project (astar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

include_directories(source)
add_library(astar STATIC
    source/AStar.cpp
    source/AStar2.cpp
    source/util.h)

add_executable(benchmark-astar main.cpp)
target_link_libraries(benchmark-astar astar /usr/local/lib/libbenchmark.a pthread)

