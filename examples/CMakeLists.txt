cmake_minimum_required(VERSION 3.5)
project(example-1-sphere)

find_package(Threads REQUIRED)

add_executable(example1sphere "example-1-sphere/main.cpp")

target_link_libraries(
    example1sphere
    cnearest_lib
    )


add_executable(example2prolateSpheroid "example-2-prolate-spheroid/main.cpp")

target_link_libraries(
    example2prolateSpheroid
    cnearest_lib
    )
    
find_package(Kokkos)

if(Kokkos_FOUND)

    add_executable(kokkosexample1sphere "kokkos-example-1-sphere/main.cpp")

    target_link_libraries(
        kokkosexample1sphere
        cnearest_lib
        kokkos_extensions
        )

else()
    message("Kokkos not found")
endif()