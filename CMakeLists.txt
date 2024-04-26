cmake_minimum_required (VERSION 3.8)

project ("cnearest")

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)


# Include sub-projects.
add_subdirectory ("src")
