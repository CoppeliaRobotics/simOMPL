cmake_minimum_required(VERSION 3.15)
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
find_package(Eigen3 REQUIRED)

include(ExternalProject)
set(OMPL_VERSION 1.5.0)
ExternalProject_Add(ompl-${OMPL_VERSION}
    URL https://github.com/ompl/ompl/archive/${OMPL_VERSION}.tar.gz
    PATCH_COMMAND sed -i.bak "s|MAX_QUATERNION_NORM_ERROR = 1e-9|MAX_QUATERNION_NORM_ERROR = 1e-6|" ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix/src/ompl-${OMPL_VERSION}/src/ompl/base/spaces/src/SO3StateSpace.cpp
    CMAKE_ARGS ${COMMON_CMAKE_ARGS} -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_PYTESTS=OFF -DOMPL_BUILD_TESTS=OFF -DOMPL_REGISTRATION=OFF
    BUILD_COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix/src/ompl-${OMPL_VERSION}-build --config ${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ""
)

set(OMPL_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix/src/ompl-${OMPL_VERSION}-build/lib/${CMAKE_SHARED_LIBRARY_PREFIX}ompl${CMAKE_SHARED_LIBRARY_SUFFIX})
set(OMPL_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix/src/ompl-${OMPL_VERSION}/src ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix/src/ompl-${OMPL_VERSION}-build/src)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${OMPL_INCLUDE_DIRS})

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simOMPL.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

coppeliasim_add_plugin(simExtOMPL SOURCES plugin.cpp)
add_dependencies(simExtOMPL ompl-${OMPL_VERSION})
target_link_libraries(simExtOMPL Boost::boost Boost::system Boost::serialization)
target_link_libraries(simExtOMPL Eigen3::Eigen)
target_link_libraries(simExtOMPL ${OMPL_LIBRARY})
coppeliasim_add_library(${OMPL_LIBRARY})
