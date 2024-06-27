cmake_minimum_required(VERSION 3.2)
project(cnearest_lib C CXX)

file(GLOB LIB_SOURCES "*.cpp")
file(GLOB SUB_LIB_SOURCES "*/*.cpp")
add_library(cnearest_lib ${LIB_SOURCES} ${SUB_LIB_SOURCES})

target_include_directories(cnearest_lib PUBLIC .)
target_include_directories(cnearest_lib PUBLIC discretisations)
target_include_directories(cnearest_lib PUBLIC nearest_neighbour)

target_link_libraries(
	cnearest_lib
	Eigen3::Eigen
    BLAS::BLAS
	LAPACK::LAPACK
    )
