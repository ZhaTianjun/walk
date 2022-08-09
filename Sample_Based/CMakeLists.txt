cmake_minimum_required(VERSION 3.10)
project(Astar)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)


add_library(lib_Astar
        src/Astar.cpp
)
add_library(lib_ARAstar
        src/ARAstar.cpp
)
add_executable(path_planner src/Planner_test.cpp)

target_link_libraries(
        path_planner
        lib_Astar
        lib_ARAstar
)