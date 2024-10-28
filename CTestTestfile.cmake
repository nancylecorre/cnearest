# CMake generated Testfile for 
# Source directory: C:/Users/Djsmi/OneDrive - University of Birmingham/Research/cnearest-2024/cnearest_RSE_project/cnearest_new
# Build directory: C:/Users/Djsmi/OneDrive - University of Birmingham/Research/cnearest-2024/cnearest_RSE_project/cnearest_new
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cnearest_test "tests/cnearest_test")
set_tests_properties(cnearest_test PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Djsmi/OneDrive - University of Birmingham/Research/cnearest-2024/cnearest_RSE_project/cnearest_new/CMakeLists.txt;25;add_test;C:/Users/Djsmi/OneDrive - University of Birmingham/Research/cnearest-2024/cnearest_RSE_project/cnearest_new/CMakeLists.txt;0;")
add_test(cnearest_matlab_tests "tests/cnearest_matlab_tests")
set_tests_properties(cnearest_matlab_tests PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Djsmi/OneDrive - University of Birmingham/Research/cnearest-2024/cnearest_RSE_project/cnearest_new/CMakeLists.txt;26;add_test;C:/Users/Djsmi/OneDrive - University of Birmingham/Research/cnearest-2024/cnearest_RSE_project/cnearest_new/CMakeLists.txt;0;")
subdirs("src")
subdirs("examples")
subdirs("tests")
