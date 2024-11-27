rock_find_cmake(Boost REQUIRED COMPONENTS thread system)

pkg_check_modules(SDFORMAT QUIET sdformat)
pkg_check_modules(SDFORMAT9 QUIET sdformat9)
pkg_check_modules(SDFORMAT12 QUIET sdformat12)
pkg_check_modules(SDFORMAT15 QUIET sdformat15)

if(${SDFORMAT_FOUND})
  set(SDFORMAT_PKGCONFIG_NAME "sdformat")
elseif(${SDFORMAT9_FOUND})
  set(SDFORMAT_PKGCONFIG_NAME "sdformat9")
elseif(${SDFORMAT12_FOUND})
  set(SDFORMAT_PKGCONFIG_NAME "sdformat12")
elseif(${SDFORMAT15_FOUND})
  set(SDFORMAT_PKGCONFIG_NAME "sdformat15")
else()
  message(SEND_ERROR "Could not find sdformat, sdformat9 or sdformat12 with pkg-config")
endif()

rock_library(kdl_parser SOURCES kdl_parser.cpp RobotModelFormat.cpp sdf.cpp
                        HEADERS kdl_parser.hpp RobotModelFormat.hpp
                        DEPS_PKGCONFIG
			  eigen3 
			  orocos-kdl 
			  base-lib 
			  tinyxml
                          urdfdom_headers 
			  urdfdom
                          ${SDFORMAT_PKGCONFIG_NAME}
)
