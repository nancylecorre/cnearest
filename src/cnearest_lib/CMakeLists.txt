cmake_minimum_required(VERSION 3.2)
project(cnearest_lib C CXX)

file(GLOB LIB_SOURCES "*.cpp")
add_library(cnearest_lib ${LIB_SOURCES})


target_link_libraries(
	cnearest_lib
	Eigen3::Eigen
    )
