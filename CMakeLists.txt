cmake_minimum_required(VERSION 2.8.11)
project(simExtOMPL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED COMPONENTS system serialization)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
    endif()
endif()
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)

set(FIND_OMPL_VIA_PKGCONFIG OFF CACHE BOOL "Use pkg-config to find OMPL")
if(FIND_OMPL_VIA_PKGCONFIG)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OMPL REQUIRED IMPORTED_TARGET ompl)
else()
    find_package(OMPL REQUIRED)
endif()

include_directories(${COPPELIASIM_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simExtOMPL.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

set(SOURCES
    plugin.cpp
    ${COPPELIASIM_EXPORTED_SOURCES}
)
add_library(simExtOMPL SHARED ${SOURCES})
target_link_libraries(simExtOMPL ${Boost_LIBRARIES} ${COPPELIASIM_LIBRARIES})
if(FIND_OMPL_VIA_PKGCONFIG)
    target_link_libraries(simExtOMPL PkgConfig::OMPL)
else()
    target_link_libraries(simExtOMPL ${OMPL_LIBRARIES})
endif()

install(TARGETS simExtOMPL DESTINATION ${COPPELIASIM_PLUGINS_DIR})

