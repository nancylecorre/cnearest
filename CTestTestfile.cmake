# CMake generated Testfile for 
# Source directory: /home/dave/Research/cnearest
# Build directory: /home/dave/Research/cnearest
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cnearest_test "tests/cnearest_test")
set_tests_properties(cnearest_test PROPERTIES  _BACKTRACE_TRIPLES "/home/dave/Research/cnearest/CMakeLists.txt;25;add_test;/home/dave/Research/cnearest/CMakeLists.txt;0;")
add_test(cnearest_matlab_tests "tests/cnearest_matlab_tests")
set_tests_properties(cnearest_matlab_tests PROPERTIES  _BACKTRACE_TRIPLES "/home/dave/Research/cnearest/CMakeLists.txt;26;add_test;/home/dave/Research/cnearest/CMakeLists.txt;0;")
subdirs("src")
subdirs("examples")
subdirs("tests")
