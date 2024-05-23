FROM ergoregion/cpp_build_tools:trial

COPY ./src ./src
COPY ./tests ./tests
COPY ./CMakeLists.txt ./CMakeLists.txt
COPY ./conanfile.txt ./conanfile.txt
COPY ./.clang-format ./.clang-format
COPY ./.clang-format-ignore ./.clang-format-ignore

RUN conan install . --build=missing --output-folder=out &&\
	cmake . -DCMAKE_TOOLCHAIN_FILE=/out/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
	

CMD cmake --build . && \
	ctest --output-on-failure &&\
	lcov -d . -c -o coverage.info &&\
	lcov -r coverage.info */c++/* */gcc/* */tests/* */gtest/* */eigen3/* -o coverageFiltered.info &&\
	genhtml coverageFiltered.info --output-directory html_coverage &&\
	lcov --list coverageFiltered.info




#CMD find src -iname *.h -o -iname *.cpp | xargs clang-format -i --dry-run