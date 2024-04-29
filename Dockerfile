FROM ergoregion/cpp_build_tools:trial

COPY ./src ./src
COPY ./tests ./tests
COPY ./CMakeLists.txt ./CMakeLists.txt
COPY ./conanfile.txt ./conanfile.txt

CMD conan install . --build=missing --output-folder=out &&\
	cmake . -DCMAKE_TOOLCHAIN_FILE=/out/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release&& \
	cmake --build . && \
	ctest --output-on-failure


