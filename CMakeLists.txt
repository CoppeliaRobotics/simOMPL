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
set(OMPL_ABI_VERSION 16)
set(OMPL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix)
set(OMPL_SRC_DIR ${OMPL_PREFIX}/src/ompl-${OMPL_VERSION})
set(OMPL_BUILD_DIR ${OMPL_PREFIX}/src/ompl-${OMPL_VERSION}-build)
ExternalProject_Add(ompl-${OMPL_VERSION}
    URL https://github.com/ompl/ompl/archive/${OMPL_VERSION}.tar.gz
    PATCH_COMMAND sed -i.bak "s|MAX_QUATERNION_NORM_ERROR = 1e-9|MAX_QUATERNION_NORM_ERROR = 1e-6|" ${OMPL_SRC_DIR}/src/ompl/base/spaces/src/SO3StateSpace.cpp
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_PYTESTS=OFF -DOMPL_BUILD_TESTS=OFF -DOMPL_REGISTRATION=OFF -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    BUILD_COMMAND ${CMAKE_COMMAND} --build ${OMPL_BUILD_DIR} --config ${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ""
)

include_directories(${OMPL_SRC_DIR}/src ${OMPL_BUILD_DIR}/src)
link_directories(${OMPL_BUILD_DIR}/lib)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simOMPL.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

coppeliasim_add_plugin(simExtOMPL SOURCES plugin.cpp)
add_dependencies(simExtOMPL ompl-${OMPL_VERSION})
target_link_libraries(simExtOMPL Boost::boost Boost::system Boost::serialization)
target_link_libraries(simExtOMPL Eigen3::Eigen)
target_link_libraries(simExtOMPL ompl)
if(NOT WIN32)
    install(DIRECTORY ${OMPL_BUILD_DIR}/lib/ DESTINATION ${COPPELIASIM_LIBRARIES_DIR})
endif()
