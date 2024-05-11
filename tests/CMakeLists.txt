cmake_minimum_required(VERSION 3.5)
project(cnearestlib_tests)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

include_directories(../src)

file(GLOB TEST_SOURCES "src/*.cpp")
add_executable(cnearest_test ${TEST_SOURCES})

target_link_libraries(
    cnearest_test
    cnearest_lib
    gtest::gtest
    Eigen3::Eigen
    Threads::Threads
    )