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
list(APPEND CMAKE_MODULE_PATH
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

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${OMPL_INCLUDE_DIRS})

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simOMPL.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

coppeliasim_add_plugin(simExtOMPL SOURCES plugin.cpp)
target_link_libraries(simExtOMPL Boost::boost Boost::system Boost::serialization)
if(FIND_OMPL_VIA_PKGCONFIG)
    target_link_libraries(simExtOMPL PkgConfig::OMPL)
else()
    target_link_libraries(simExtOMPL ${OMPL_LIBRARIES})
endif()
