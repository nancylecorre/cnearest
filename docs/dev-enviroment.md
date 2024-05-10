# Setting up a developement enviroment

We have cmake files to build and test this libray, this will require a working c++ comiler and cmake. We also rely on some dependencies.

## Dependencies

The library depends on some prerequisite packages, including blas, lapack, eigen and gtest.
We use cmake commands to detect these, so provided they can be detected the library can be build.
However to assist versioning and detection we recommend using conan to install and locate the correct versions of the dependencies.

## Linux

Aquire some comiler for c++ and cmake eg

    apt update && apt install -y \
    cmake\
    build-essential

Aquire conan (as descibed on their own pages), this is usually done using pip
You will also need to make a conan profile (principally a choise of compiler), this can be autodetected

    apt update && apt install -y python3-pip
    pip3 install --no-cache-dir conan
    conan profile detect

use conan to install the dependencies of this individual project

    conan install conanfile.txt --build=missing --output-folder=out

This should install the requiremenat defined in the file, it should alse create a conan_toolchain.cmake file  (in the out directory) which contains all the inforrmation which cmake needs to find the installed packages

Use cmake to build the code (with an argument telling it about the conan files we have installed)

    cmake . -DCMAKE_TOOLCHAIN_FILE=/out/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
    cmake --build .

Run the unit tests with ctest

    ctest --output-on-failure

## Windows

Aquire conan (as descibed on their own pages), this is usually done using pip
Install a c++ compiler of your choise (eg visual studio)

install the dependencies defined in the conanfile, using the command,

    conan install conanfile.txt --build=missing --output-folder=out

This should install the requiremenat defined in the file, it should alse create a conan_toolchain.cmake file  (in the out directory) which contains all the inforrmation which cmake needs to find the installed packages

Now you use the cmake system with the additional argument

    "CMAKE_TOOLCHAIN_FILE": "out/conan_toolchain.cmake", 
to locate the dependencies.
Use cmake to perform your build and test
Eg for visual studio 2022, something like

    cmake . -G "Visual Studio 17 2022"  -DCMAKE_TOOLCHAIN_FILE="out/conan_toolchain.cmake" -DCMAKE_POLICY_DEFAULT_CMP0091=NEW
    cmake --build .
    ctest --output-on-failure

Should build and test the library

Alternativly visual studio has cmake integreations which can be used.
