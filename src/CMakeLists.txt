rock_find_cmake(Boost REQUIRED COMPONENTS thread system)

rock_library(kdl_parser SOURCES kdl_parser.cpp sdf.cpp
                        HEADERS kdl_parser.hpp
                        DEPS_PKGCONFIG 
			  eigen3 
			  orocos-kdl 
			  base-lib 
			  tinyxml
                          urdfdom_headers 
			  urdfdom
                          sdformat
)
