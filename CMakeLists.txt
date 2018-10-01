cmake_minimum_required(VERSION 2.4)
project(RPLidarINTechLib CXX)

set(CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE SOURCES "./include/*.h" "./include/*.hpp" "./src/*.c" "./src/*.cpp")
include_directories(include)
add_executable(RPLidarINTechLib ${SOURCES})