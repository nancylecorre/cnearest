cmake_minimum_required(VERSION 3.5)
project(example-1-sphere)

find_package(Threads REQUIRED)

add_executable(example1sphere "example-1-sphere/main.cpp")

target_link_libraries(
    example1sphere
    cnearest_lib
    cnearest_extensions
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )


add_executable(example2prolateSpheroid "example-2-prolate-spheroid/main.cpp")

target_link_libraries(
    example2prolateSpheroid
    cnearest_lib
    cnearest_extensions
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )
    
find_package(Kokkos)

if(Kokkos_FOUND)

add_executable(example3spherekokkos "example-3-sphere-kokkos/main.cpp")

target_link_libraries(
    example3spherekokkos
    cnearest_lib
    cnearest_extensions
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )


add_executable(example5mandlebrook "example-5-mandlebrook/main.cpp")

target_link_libraries(
    example5mandlebrook
    cnearest_lib
    cnearest_extensions
    )
else()
message("Kokkos not found")
endif()