FROM ergoregion/cpp_build_tools:trial

COPY ./src ./src
COPY ./tests ./tests
COPY ./examples ./examples
COPY ./CMakeLists.txt ./CMakeLists.txt
COPY ./conanfile.txt ./conanfile.txt
COPY ./.clang-format ./.clang-format
COPY ./.clang-format-ignore ./.clang-format-ignore

RUN conan install . --build=missing --output-folder=out &&\
	cmake . -DCMAKE_TOOLCHAIN_FILE=/out/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
	
RUN cmake --build .

CMD examples/example1sphere




#CMD find src -iname *.h -o -iname *.cpp | xargs clang-format -i --dry-run