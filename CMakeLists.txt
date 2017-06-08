cmake_minimum_required(VERSION 2.8.11)
project(v_repExtOMPL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
find_package(VREP REQUIRED)
find_package(OMPL REQUIRED)

include_directories(${VREP_INCLUDE})
include_directories(${CMAKE_BINARY_DIR}/generated)
include_directories(${CMAKE_SOURCE_DIR}/external/ompl/src)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/generated/stubs.cpp ${CMAKE_BINARY_DIR}/generated/stubs.h ${CMAKE_BINARY_DIR}/generated/lua_calltips.cpp
    COMMAND python ${CMAKE_SOURCE_DIR}/external/v_repStubsGen/generate.py --xml-file ${CMAKE_SOURCE_DIR}/callbacks.xml --lua-file ${CMAKE_SOURCE_DIR}/simExtOMPL.lua --gen-all ${CMAKE_BINARY_DIR}/generated
    DEPENDS ${CMAKE_SOURCE_DIR}/callbacks.xml)

set(SOURCES v_repExtOMPL.cpp)
add_library(v_repExtOMPL SHARED ${SOURCES} ${CMAKE_BINARY_DIR}/generated/stubs.cpp "${VREP_COMMON}/v_repLib.cpp")
target_link_libraries(v_repExtOMPL ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${VREP_LIBRARIES})
