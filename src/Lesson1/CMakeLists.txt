cmake_minimum_required(VERSION 3.9)
project(depth)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(depth main.cpp)
target_link_libraries (depth ${PCL_LIBRARIES})