cmake_minimum_required(VERSION 3.10)
project(ekf_2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(ekf ekf.cpp)
target_link_libraries(ekf Eigen3::Eigen)

add_executable(ekf_2d main.cpp)
target_link_libraries(ekf_2d ekf)



