cmake_minimum_required(VERSION 3.10)
project(mavix CXX)

message( "MAVIX | Open Map Server v1")
message( "---------------------------")
message(STATUS "MAVIX LIB DIR : ${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(ABSL_PROPAGATE_CXX_STD ON)

option(LIB_MAVIX_BUILD_DOC "Build documentation" OFF)
option(LIB_MAVIX_SANITIZE_ADDRESS "SANITIZE ADDRESS" OFF)
option(LIB_MAVIX_USE_CATCH ON)
option(LIB_MAVIX_USE_LIB ON)
option(LIB_MAVIX_USE_TEST OFF)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "PROFILE: ${CMAKE_BUILD_TYPE}")

# Compile Def Macro
set(MAVIX_COMPILE_DEF)

# Activate Debug Trace
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND MAVIX_COMPILE_DEF MAVIX_DEBUG_CORE)
endif()

# Available trace per object
# Use below macro in the code, not in CMAKE
# in the cmake MAVIX_DEBUG_CORE is adequate
#
# MAVIX_DEBUG_SHARED_BUFFER -- Print debug state of SharedBuffer<T>
# MAVIX_DEBUG_STREAM -- Print debug state of Stream
# MAVIX_DEBUG_MEMORY_ALLOCATOR -- Print debug state of MemoryAllocator<T>

# 0 - std allocator
# 1 - Google Arena (TcMalloc)
# 2 - JEMalloc
option(LIB_MAVIX_MEM_ALLOCATOR_TYPE "Memory Allocator Type" 1)

set(LIB_MAVIX_BUILD_DOC OFF)
set(LIB_MAVIX_SANITIZE_ADDRESS OFF)
set(LIB_MAVIX_USE_CATCH ON)
set(LIB_MAVIX_USE_LIB ON)
set(LIB_MAVIX_USE_TEST OFF)
set(LIB_MAVIX_MEM_ALLOCATOR_TYPE 1)

# Add ASAN
if(LIB_MAVIX_SANITIZE_ADDRESS)
    set(SANITIZE_ADDRESS ON)
endif(LIB_MAVIX_SANITIZE_ADDRESS)

#  make ourself sanitize and sane well.
if(NOT SANITIZE_ADDRESS)
 find_package(Sanitizers)
endif()

# Add Doxygen
if(LIB_MAVIX_BUILD_DOC)
    if(NOT DOXYGEN_FOUND)
        find_package(Doxygen)
    endif()

    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_SOURCE_DIR}/generated/docs-cached/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

         # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_mavix ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/generated/docs-cached/
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(LIB_MAVIX_BUILD_DOC)

# Add Unit Test Lib Catch2
if(LIB_MAVIX_USE_CATCH)
 find_package(Catch2 3 REQUIRED)
 if(Catch2_FOUND)
    message(STATUS  "CATCH2 : Found")
 else()
    message(STATUS  "CATCH2 : Not Found")
 endif()
endif()

# Add dependencies source code

# Add support for mavix memory allocator using jemalloc or google-arena (tcmalloc) support
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/memory-allocator-build.cmake)

find_library(JEMALLOC_LIBRARY NAMES jemalloc libjemalloc PATHS ${CMAKE_CURRENT_SOURCE_DIR}/deps/jemalloc-5.3.0/lib)
find_library(TCMALLOC_LIBRARY NAMES tcmalloc libtcmalloc PATHS ${CMAKE_CURRENT_SOURCE_DIR}/deps/gperftools-2.15/.libs)

if(NOT JEMALLOC_LIBRARY)
    # Build jemalloc
    message("Build: JEMalloc from source")
    JEMallocBuild(
        SOURCE_DIR "${CMAKE_SOURCE_DIR}/deps/jemalloc-5.3.0"
        PREFIX "${CMAKE_CURRENT_BINARY_DIR}/jemalloc_build"
    )
endif()

if(NOT TCMALLOC_LIBRARY)
    # Build TCMalloc
    message("Build: TCMalloc from source")
    TCMallocBuild(
        SOURCE_DIR "${CMAKE_SOURCE_DIR}/deps/gperftools-2.15"
        PREFIX "${CMAKE_CURRENT_BINARY_DIR}/tcmalloc_build"
    )
endif()

if(LIB_MAVIX_MEM_ALLOCATOR_TYPE EQUAL 2 OR LIB_MAVIX_MEM_ALLOCATOR_TYPE EQUAL 1)
    # Create interface libraries
    add_library(JEMallocInterface INTERFACE)
    add_dependencies(JEMallocInterface JEMallocProject)

    add_library(TCMallocInterface INTERFACE)
    add_dependencies(TCMallocInterface TCMallocProject)

    set(JEMALLOC_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/deps/jemalloc-5.3.0/include")
    set(JEMALLOC_LIB "${CMAKE_CURRENT_SOURCE_DIR}/deps/jemalloc-5.3.0/lib/libjemalloc.a")

    set (TCMALLOC_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/deps/gperftools-2.15/src")
    set (TCMALLOC_LIB "${CMAKE_CURRENT_SOURCE_DIR}/deps/gperftools-2.15/.libs/libtcmalloc.a")
endif()

if(LIB_MAVIX_MEM_ALLOCATOR_TYPE EQUAL 1)
    list(APPEND MAVIX_COMPILE_DEF MAVIX_ALLOCATOR_GOOGLE_ARENA)
elseif(LIB_MAVIX_MEM_ALLOCATOR_TYPE EQUAL 2)
    list(APPEND MAVIX_COMPILE_DEF MAVIX_ALLOCATOR_JEMALLOC)
endif()

message(STATUS "JEMALLOC LIB: ${JEMALLOC_LIBRARY}")
message(STATUS "TCMALLOC LIB: ${TCMALLOC_LIBRARY}")

if(MAVIX_COMPILE_DEF)
add_compile_definitions(${MAVIX_COMPILE_DEF})
message(STATUS "Mavix compile flags: ${MAVIX_COMPILE_DEF}" )
endif()

find_package(Protobuf CONFIG REQUIRED)

# Google Abseil
message(STATUS "Abseil : Configured")
add_subdirectory(deps/abseil-20230802.1 build-abseil)

# UTF8-CPP source code
message(STATUS "Utf8Cpp : Configured")
file(GLOB_RECURSE SOURCES_UTF8CPP CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/utfcpp-4.0.4/source/*.h
)

add_library(utf8cpp ${SOURCES_UTF8CPP})
set_target_properties(utf8cpp PROPERTIES LINKER_LANGUAGE CXX)
target_compile_features(utf8cpp PUBLIC cxx_std_17)
target_include_directories(utf8cpp
    PUBLIC
        deps/utfcpp-4.0.4/source/
)

# Add zlib from deps/zlib
message(STATUS "zlib : Configured")
add_subdirectory(deps/zlib-1.2.11 build-zlib)

# Add lz4 from deps/lz4-1.9.4
message(STATUS "lz4 : Configured")
set (${LZ4_BUNDLED_MODE} ON)
add_subdirectory(deps/lz4-1.9.4/build/cmake build-lz4)

# Add tinyxml from deps/tinyxml2-9.0.0
message(STATUS "TinyXML : Configured")
add_subdirectory(deps/tinyxml2-9.0.0 build-tinyxml2)

# nvm nvm::core
add_subdirectory(deps/nvm-core build-nvm-core)

# nvm mavix::core
add_subdirectory(libs/mavix-core build-mavix-core)

# nvm mavix::utils
add_subdirectory(libs/mavix-utils build-mavix-utils)

# nvm mavix::osm
add_subdirectory(libs/mavix-osm build-mavix-osm)

# nvm mavix::storage
add_subdirectory(libs/mavix-storage build-mavix-storage)

# nvm mavix::spatial
add_subdirectory(libs/mavix-spatial build-mavix-spatial)

# nvm mavix::tiles
add_subdirectory(libs/mavix-tiles build-mavix-tiles)

# nvm mavix::renderer
add_subdirectory(libs/mavix-renderer build-mavix-renderer)



# Main headers and sources
file(GLOB_RECURSE SOURCES_MAVIX CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/mavix/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mavix/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/mavix/*.s
)




add_library(${PROJECT_NAME}  ${SOURCES_MAVIX} )
target_link_libraries(${PROJECT_NAME} 
    utf8cpp
    nvm-core
    absl::base 
    absl::flat_hash_map 
    absl::algorithm
    absl::numeric
    absl::random_random
    absl::synchronization 
    absl::strings
    absl::time
    absl::utility)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        mavix/
)


add_executable(${PROJECT_NAME}-server main.cc)
if(SANITIZE_ADDRESS AND Sanitizers_FOUND)
    add_sanitizers("${PROJECT_NAME}-server")
    message(STATUS "${PROJECT_NAME}-server_LIB::ASAN:ON.")
endif()

target_link_libraries(${PROJECT_NAME}-server PRIVATE  ${PROJECT_NAME} )
target_include_directories("${PROJECT_NAME}-server"
    PUBLIC
        mavix/
)

if(LIB_MAVIX_USE_TEST)
    message(STATUS "TEST::ADD_UNIT_TEST.")
    add_subdirectory(tests)
endif()       

message(STATUS "Mavix project configuration done\n")
set(mavix_FOUND  ON)