rock_library(kdl_parser SOURCES kdl_parser.cpp
                        HEADERS kdl_parser.hpp
                        DEPS_PKGCONFIG orocos-kdl urdf
                        DEPS_PKGCONFIG eigen3
)
