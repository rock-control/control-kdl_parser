configure_file(TestConfig.hpp.in TestConfig.hpp @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
rock_testsuite(test_suite suite.cpp
    test_sdf.cpp test_RobotModelFormat.cpp DEPS kdl_parser)
