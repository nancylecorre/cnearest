cmake_minimum_required(VERSION 3.5)
project(example-1-sphere)

find_package(Threads REQUIRED)

add_executable(example1sphere "example-1-sphere/main.cpp")

target_link_libraries(
    example1sphere
    cnearest_lib
    Eigen3::Eigen
    Threads::Threads
    )