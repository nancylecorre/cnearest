cmake_minimum_required(VERSION 3.8)
project(example-1-sphere)

find_package(Threads REQUIRED)

add_executable(example1sphere "example-1-sphere/main.cpp")

target_link_libraries(
    example1sphere
    cnearest_lib
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )


add_executable(example2prolateSpheroid "example-2-prolate-spheroid/main.cpp")

target_link_libraries(
    example2prolateSpheroid
    cnearest_lib
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )