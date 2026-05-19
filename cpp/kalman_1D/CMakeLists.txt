cmake_minimum_required(VERSION 3.10)
project(kalman)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3)

add_library(kalman_lib kalman.cpp)
target_link_libraries(kalman_lib Eigen3::Eigen)

add_executable(kalman main.cpp)
target_link_libraries(kalman kalman_lib)