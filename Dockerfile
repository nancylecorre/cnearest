FROM ergoregion/cpp_build_tools:trial

COPY . .

CMD cmake --version && \
	ls && \
	cmake . && \
	cmake --build . && \
	ctest --output-on-failure


