cmake_minimum_required(VERSION 4.1)
project(dynamic_environment)

set(CMAKE_CXX_STANDARD 20)

add_executable(dynamic_environment main.cpp
        Field.cpp
        Field.h
        heuristic_functions.cpp
        heuristic_functions.h
        Astar.cpp
        Astar.h
        DynamicObstacle.cpp
        DynamicObstacle.h
        DstarLite.cpp
        DstarLite.h)
