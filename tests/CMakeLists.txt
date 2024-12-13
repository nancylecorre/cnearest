cmake_minimum_required(VERSION 3.5)
project(cnearestlib_tests)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

file(GLOB TEST_SOURCES "src/*.cpp")
add_executable(cnearest_test ${TEST_SOURCES})

target_link_libraries(
    cnearest_test
    cnearest_lib
    GTest::GTest
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )

project(cnearest_matlab_tests)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

file(GLOB TEST_SOURCES "matlab_test/*.cpp")
add_executable(cnearest_matlab_tests ${TEST_SOURCES})

target_link_libraries(
    cnearest_matlab_tests
    cnearest_lib
    GTest::GTest
    Eigen3::Eigen
    BLAS::BLAS
    LAPACK::LAPACK
    Threads::Threads
    )
