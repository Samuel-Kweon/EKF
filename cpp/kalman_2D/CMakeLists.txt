cmake_minimum_required(VERSION 3.10)
project(kalman_2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(Kalman Kalman.cpp)
target_link_libraries(Kalman Eigen3::Eigen)

add_executable(kalman_2d main.cpp)
target_link_libraries(kalman_2d Kalman)
