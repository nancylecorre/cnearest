# Setting up a development environment

We have CMake files to build and test this library, this will require a working C++ compiler and CMake. We also rely on some dependencies.

## Dependencies

The library depends on some prerequisite packages, including BLAS, LAPACK, Eigen and gtest.
We use `cmake` commands to detect these, so provided they can be detected the library can be build.
However to assist versioning and detection we recommend using conan to install and locate the correct versions of the dependencies.

## Linux

Acquire some compiler for C++ and CMake eg

    apt update && apt install -y \
    cmake\
    build-essential

Acquire conan (as described on their own pages), this is usually done using pip
You will also need to make a conan profile (principally a choice of compiler), this can be autodetected

    apt update && apt install -y python3-pip
    pip3 install --no-cache-dir conan
    conan profile detect

use conan to install the dependencies of this individual project

    conan install conanfile.txt --build=missing --output-folder=out -s build_type=Release

This should install the requirements defined in the file, it should also create a conan_toolchain.cmake file  (in the out directory) which contains all the information which cmake needs to find the installed packages

Use cmake to build the code (with an argument telling it about the conan files we have installed)

    cmake . -DCMAKE_TOOLCHAIN_FILE=/out/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
    cmake --build .

Run the unit tests with CTest

    ctest --output-on-failure

## Windows

Acquire conan (as described on their own pages). This is usually done using pip.

Install a C++ compiler of your choice (e.g., Visual Studio).

Use the command

    conan profile detect

to set up a default conan profile. You should see output to the console similar to

    [settings]
    arch=x86_64
    build_type=Release
    compiler=msvc
    compiler.cppstd=17
    compiler.runtime=dynamic
    compiler.version=193
    os=Windows

Note the cNEAREST requires C++17 language features. If you see `compiler.cppstd=14` in the above, you will need to modify
your default conan profile directly to update this to `17` as above. Run `conan profile default path` to find the location
of the file to modify.

Install the dependencies defined in the conanfile, using the command,

    conan install conanfile.txt --build=missing --output-folder=out -s build_type=Release

This should install the requirements defined in the file, it should also create a conan_toolchain.cmake file  (in the out directory) which contains all the information which CMake needs to find the installed packages.

Now you use the `cmake` command with the additional argument

    "CMAKE_TOOLCHAIN_FILE": "out/conan_toolchain.cmake", 

to locate the dependencies:

    cmake . -G "Visual Studio 17 2022"  -DCMAKE_TOOLCHAIN_FILE="out/conan_toolchain.cmake" -DCMAKE_POLICY_DEFAULT_CMP0091=NEW -DCMAKE_BUILD_TYPE=Release

Use CMake to perform your build and test.
E.g., for Visual Studio 2022, something like:

    cmake --build .
    ctest --output-on-failure

should build and test the library.

Alternatively visual studio has cmake integrations which can be used.
