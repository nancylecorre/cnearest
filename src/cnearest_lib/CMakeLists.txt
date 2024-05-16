cmake_minimum_required(VERSION 3.2)
project(cnearest_lib C CXX)

file(GLOB LIB_SOURCES "*.cpp")
file(GLOB D_LIB_SOURCES "discretisations/*.cpp")
add_library(cnearest_lib ${LIB_SOURCES} ${D_LIB_SOURCES})

target_include_directories(cnearest_lib PUBLIC .)
target_include_directories(cnearest_lib PUBLIC discretisations)

target_link_libraries(
	cnearest_lib
	Eigen3::Eigen
    )
