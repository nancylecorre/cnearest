cmake_minimum_required (VERSION 3.8)

project ("cnearest")

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Include sub-projects.
add_subdirectory (src)
add_subdirectory(tests)

include(CTest)
enable_testing()
add_test(cnearest_test tests/cnearest_test)
