cmake_minimum_required(VERSION 3.16.3)
project(simOMPL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED COMPONENTS serialization)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED COMPONENTS simLib-1 simPlusPlus-1)
find_package(Eigen3 REQUIRED)

include(ExternalProject)
set(OMPL_VERSION 1.7.0)
set(OMPL_ABI_VERSION 18)
set(OMPL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ompl-${OMPL_VERSION}-prefix)
set(OMPL_SRC_DIR ${OMPL_PREFIX}/src/ompl-${OMPL_VERSION})
set(OMPL_BUILD_DIR ${OMPL_PREFIX}/src/ompl-${OMPL_VERSION}-build)
ExternalProject_Add(ompl-${OMPL_VERSION}
    URL https://github.com/ompl/ompl/archive/${OMPL_VERSION}.tar.gz
    PATCH_COMMAND
        ${CMAKE_COMMAND} -E echo "Applying patch to remove Boost::system..." &&
        patch -p1 < ${CMAKE_SOURCE_DIR}/ompl-1.7.0-remove-boost-system.patch
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_PYTESTS=OFF -DOMPL_BUILD_TESTS=OFF -DOMPL_REGISTRATION=OFF -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    BUILD_COMMAND ${CMAKE_COMMAND} --build ${OMPL_BUILD_DIR} --config ${CMAKE_BUILD_TYPE}
    INSTALL_COMMAND ""
)

include_directories(${OMPL_SRC_DIR}/src ${OMPL_BUILD_DIR}/src)
if(WIN32)
    set(OMPL_LIB_DIR "${OMPL_BUILD_DIR}/src/ompl/${CMAKE_BUILD_TYPE}")
else()
    if(EXISTS "${OMPL_BUILD_DIR}/lib" AND IS_DIRECTORY "${OMPL_BUILD_DIR}/lib")
        set(OMPL_LIB_DIR "${OMPL_BUILD_DIR}/lib")
    else()
        set(OMPL_LIB_DIR "${OMPL_BUILD_DIR}/src/ompl")
    endif()
endif()
link_directories(${OMPL_LIB_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/sourceCode/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lua/simOMPL.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/sourceCode/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

coppeliasim_add_plugin(simOMPL SOURCES sourceCode/plugin.cpp)
add_dependencies(simOMPL ompl-${OMPL_VERSION})
target_link_libraries(simOMPL PRIVATE Boost::boost Boost::serialization)
target_link_libraries(simOMPL PRIVATE Eigen3::Eigen)
target_link_libraries(simOMPL PRIVATE ompl)
if(NOT WIN32)
    file(GLOB OMPL_LIBS "${OMPL_LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}ompl*${CMAKE_SHARED_LIBRARY_SUFFIX}*")
    install(FILES ${OMPL_LIBS} DESTINATION ${COPPELIASIM_LIBRARIES_DIR})
endif()
