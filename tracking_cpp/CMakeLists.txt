cmake_minimum_required(VERSION 3.16)

project(drone_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(MySim main.cpp)
target_link_libraries(MySim Eigen3::Eigen)

