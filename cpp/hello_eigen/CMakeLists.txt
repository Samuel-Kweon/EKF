cmake_minimum_required(VERSION 3.10)
project(hello_eigen)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(hello_eigen main.cpp)
target_link_libraries(hello_eigen Eigen3::Eigen)