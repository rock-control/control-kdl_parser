rock_find_cmake(Boost REQUIRED COMPONENTS thread system)

rock_library(kdl_parser SOURCES kdl_parser.cpp RobotModelFormat.cpp sdf.cpp
                        HEADERS kdl_parser.hpp RobotModelFormat.hpp
                        DEPS_PKGCONFIG 
			  eigen3 
			  orocos-kdl 
			  base-lib 
			  tinyxml
                          urdfdom_headers 
			  urdfdom
                          sdformat
)
