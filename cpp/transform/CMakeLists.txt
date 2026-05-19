cmake_minimum_required(VERSION 3.10)
project(transform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(transforms_lib transforms.cpp)
target_link_libraries(transforms_lib Eigen3::Eigen)


add_executable(transform main.cpp)
target_link_libraries(transform transforms_lib)
