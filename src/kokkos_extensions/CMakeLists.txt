cmake_minimum_required(VERSION 3.2)
project(kokkos_extensions C CXX)

find_package(Kokkos)
if(Kokkos_FOUND)
    file(GLOB LIB_SOURCES "*.cpp")

    add_library(kokkos_extensions ${LIB_SOURCES})
    target_compile_definitions(kokkos_extensions PUBLIC EIGEN_NO_CUDA)
    target_include_directories(kokkos_extensions PUBLIC .)

    target_link_libraries(
        kokkos_extensions
        cnearest_lib
        Eigen3::Eigen
        BLAS::BLAS
        LAPACK::LAPACK
        Kokkos::kokkos
        )

else()
    message("Kokkos not found")
endif()