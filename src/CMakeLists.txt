rock_find_cmake(Boost REQUIRED COMPONENTS thread system)

rock_library(kdl_parser SOURCES kdl_parser.cpp sdf.cpp
                        HEADERS kdl_parser.hpp 
                        DEPS_PKGCONFIG eigen3 orocos-kdl base-lib tinyxml sdformat
                        DEPS_CMAKE urdfdom_headers urdfdom)
                        
rock_executable(check_kdl_parser
                SOURCES check_kdl_parser.cpp kdl_parser.cpp sdf.cpp
                DEPS_PKGCONFIG eigen3 orocos-kdl base-lib tinyxml sdformat
                DEPS_CMAKE urdfdom_headers urdfdom)
