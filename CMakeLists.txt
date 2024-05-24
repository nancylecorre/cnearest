cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 20)

if (WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive-")
else (WIN32)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} --coverage")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif (WIN32)

project ("cnearest")

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Include sub-projects.
add_subdirectory (src)
add_subdirectory (examples)
add_subdirectory(tests)

include(CTest)
enable_testing()
add_test(cnearest_test tests/cnearest_test)
