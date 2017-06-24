cmake_minimum_required(VERSION 2.8.12)

if(NOT VREP_ROOT)
    if(NOT DEFINED ENV{VREP_ROOT})
        if(EXISTS "${CMAKE_SOURCE_DIR}/../../programming/include")
            get_filename_component(VREP_PROGRAMMING ${CMAKE_SOURCE_DIR} PATH)
            get_filename_component(VREP_ROOT ${VREP_PROGRAMMING} PATH)
        else()
            message(FATAL_ERROR "Cannot find V-REP installation. Please set the VREP_ROOT environment variable to point to the root of your V-REP installation.")
        endif()
    else()
        set(VREP_ROOT "$ENV{VREP_ROOT}")
    endif()
endif()

file(TO_CMAKE_PATH "${VREP_ROOT}" VREP_ROOT)

if(EXISTS "${VREP_ROOT}/programming/include" AND EXISTS "${VREP_ROOT}/programming/common")
    set(VREP_FOUND TRUE)
    if(NOT VREP_FIND_QUIETLY)
        message(STATUS "Found V-REP installation at ${VREP_ROOT}.")
    endif()
    set(VREP_INCLUDE "${VREP_ROOT}/programming/include")
    set(VREP_COMMON "${VREP_ROOT}/programming/common")
else()
    if(VREP_FIND_REQUIRED)
        message(FATAL_ERROR "The specified VREP_ROOT dir does not point to a valid V-REP installation.")
    endif()
endif()

set(VREP_EXPORTED_SOURCES "${VREP_COMMON}/v_repLib.cpp")

if(WIN32)
    add_definitions(-DWIN_VREP)
    add_definitions(-DNOMINMAX)
    add_definitions(-Dstrcasecmp=_stricmp)
    set(VREP_LIBRARIES shlwapi)
elseif(UNIX AND NOT APPLE)
    add_definitions(-DLIN_VREP)
    set(VREP_LIBRARIES "")
elseif(UNIX AND APPLE)
    add_definitions(-DMAC_VREP)
    set(VREP_LIBRARIES "")
endif()

function(VREP_GENERATE_STUBS GENERATED_OUTPUT_DIR)
    cmake_parse_arguments(VREP_GENERATE_STUBS "" "XML_FILE;LUA_FILE" "" ${ARGN})
    if("${VREP_GENERATE_STUBS_LUA_FILE}" STREQUAL "")
        add_custom_command(OUTPUT ${GENERATED_OUTPUT_DIR}/stubs.cpp ${GENERATED_OUTPUT_DIR}/stubs.h
            COMMAND python ${CMAKE_SOURCE_DIR}/external/v_repStubsGen/generate.py --xml-file ${VREP_GENERATE_STUBS_XML_FILE} --gen-all ${GENERATED_OUTPUT_DIR}
            DEPENDS ${VREP_GENERATE_STUBS_XML_FILE})
    else()
        add_custom_command(OUTPUT ${GENERATED_OUTPUT_DIR}/stubs.cpp ${GENERATED_OUTPUT_DIR}/stubs.h ${GENERATED_OUTPUT_DIR}/lua_calltips.cpp
            COMMAND python ${CMAKE_SOURCE_DIR}/external/v_repStubsGen/generate.py --xml-file ${VREP_GENERATE_STUBS_XML_FILE} --lua-file ${VREP_GENERATE_STUBS_LUA_FILE} --gen-all ${GENERATED_OUTPUT_DIR}
            DEPENDS ${VREP_GENERATE_STUBS_XML_FILE})
    endif()
    set(VREP_EXPORTED_SOURCES ${VREP_EXPORTED_SOURCES} "${GENERATED_OUTPUT_DIR}/stubs.cpp")
endfunction(VREP_GENERATE_STUBS)

