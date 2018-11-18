cmake_minimum_required(VERSION 2.4)
project(RPLIDAR_A2 CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchkp -pedantic")

file(GLOB_RECURSE SOURCES "./*/include/*.h" "./*/include/*.hpp" "./*/src/*.c" "./*/src/*.cpp")
include_directories(include)
add_executable(rplidar_a2 ${SOURCES} main.cpp lidar/include/Measurement.h)