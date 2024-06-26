cmake_minimum_required(VERSION 3.10)
project(mavix-osm-v1 CXX)

# message( "MAVIX::OSM v1")
# message( "-----------------------------")
# message(STATUS "MAVIX::OSM v1 LIB DIR:${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(ABSL_PROPAGATE_CXX_STD ON)

option(LIB_MAVIX_OSM_BUILD_DOC "Build documentation" OFF)
option(LIB_MAVIX_OSM_SANITIZE_ADDRESS "SANITIZE ADDRESS" ON)
option(LIB_MAVIX_OSM_USE_CATCH ON)
option(LIB_MAVIX_OSM_USE_LIB ON)
option(LIB_MAVIX_OSM_USE_TEST OFF)

# Add ASAN
if(LIB_MAVIX_OSM_SANITIZE_ADDRESS)
    set(SANITIZE_ADDRESS ON)
endif(LIB_MAVIX_OSM_SANITIZE_ADDRESS)

#  make ourself sanitize and sane well.
if(NOT Sanitizers_FOUND)
 find_package(Sanitizers)
endif()

# Add Doxygen
if(LIB_MAVIX_OSM_BUILD_DOC)
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
        add_custom_target( doc_mavix_osm ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/generated/docs-cached/
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(LIB_MAVIX_OSM_BUILD_DOC)

# Add Unit Test Lib Catch2
if(LIB_MAVIX_OSM_USE_CATCH)
 message(STATUS  "CATCH2::ADD_LIB.")
 find_package(Catch2 3 REQUIRED)
endif()

# Add dependencies source code

# Generate OSM protobuffer source code


include(protobuf-generate)

set(OSM_PROTO_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/osmpbf/)
set(OSM_PROTO_IMP ${CMAKE_CURRENT_SOURCE_DIR}/proto/)
set(OSM_PROTO_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/fileformat.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/osmformat.proto
)

# add_library(osmpbf OBJECT "${OSM_PROTO_SRC}")
# target_link_libraries(osmpbf PUBLIC protobuf::libprotobuf)
# target_include_directories(osmpbf PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")

# protobuf_generate(
#     TARGET osmpbf
#     IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/proto/"
#     PROTOC_OUT_DIR "${OSM_PROTO_OUTPUT_DIR}")

# copy osm sample to build dir
set(OSM_EXAMPLE_DATA
    GranFileBlock1-deflate.osm.pbf
    GranFileBlock1-none.osm.pbf
    sample.pbf
    SimpleFileBlock1-deflate.osm.pbf
    SimpleFileBlock1-none.osm.pbf
)

# Destination folder in the build directory
set(OSM_DEST_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/osm-example/")

# Ensure the destination directory exists
file(MAKE_DIRECTORY ${OSM_DEST_FOLDER})

# Iterate over the list of files and copy each one
foreach(file IN LISTS OSM_EXAMPLE_DATA)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/${file}"  # Source path of the file
        "${OSM_DEST_FOLDER}/${file}"  # Destination path in the build directory
        COPYONLY 
    )
endforeach()

# Main headers and sources
file(GLOB_RECURSE SOURCES_MAVIX_OSM CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/osmpbf/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/osmpbf/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/osmpbf/*.s
)

add_library(${PROJECT_NAME} ${SOURCES_MAVIX_OSM} )
target_link_libraries(${PROJECT_NAME} 
    mavix-core-v1
    mavix-utils-v1
    nvm-core
    tinyxml2 
    )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        src/
        libs/
)


add_executable(${PROJECT_NAME}-test main.cc)
if(Sanitizers_FOUND)
    add_sanitizers("${PROJECT_NAME}-test")
    message(STATUS "${PROJECT_NAME}-test : ASAN OK")
endif()

target_link_libraries(${PROJECT_NAME}-test PRIVATE  ${PROJECT_NAME} )
target_include_directories("${PROJECT_NAME}-test"
    PUBLIC
        src/
        libs/
)

if(LIB_MAVIX_OSM_USE_TEST)
    message(STATUS "${PROJECT_NAME} unit test : OK")
    add_subdirectory(tests)
endif()       

message(STATUS "${PROJECT_NAME} : OK")
set(mavix-osm-v1_core_FOUND  ON)