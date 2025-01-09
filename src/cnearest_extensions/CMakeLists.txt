cmake_minimum_required(VERSION 3.2)
project(cnearest_extensions C CXX)

file(GLOB LIB_SOURCES "*.cpp")

find_package(Kokkos)
if(Kokkos_FOUND)
file(GLOB KOKKOS_LIB_SOURCES "kokkos/*.cpp")
else()
message("Kokkos not found")
endif()

add_library(cnearest_extensions ${LIB_SOURCES} ${KOKKOS_LIB_SOURCES})
target_compile_definitions(cnearest_extensions PUBLIC EIGEN_NO_CUDA)
target_include_directories(cnearest_extensions PUBLIC .)

if(Kokkos_FOUND)
target_include_directories(cnearest_extensions PUBLIC ./kokkos)
endif()

target_link_libraries(
    cnearest_extensions
    cnearest_lib
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    )

if(Kokkos_FOUND)
target_link_libraries(
    cnearest_extensions
    Kokkos::kokkos
    )
endif()