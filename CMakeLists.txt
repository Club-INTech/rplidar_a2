cmake_minimum_required(VERSION 2.4)
project(RPLidarINTechLib C)

set(CMAKE_C_STANDARD 11)

add_executable(RPLidarINTechLib main.c tcp.h LinkedValuesList.h)