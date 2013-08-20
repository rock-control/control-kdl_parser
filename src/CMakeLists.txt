rock_library(kdl_parser SOURCES kdl_parser.cpp
                        HEADERS kdl_parser.hpp
                        DEPS_PKGCONFIG eigen3 orocos-kdl
                        DEPS_CMAKE urdfdom_headers urdfdom)
