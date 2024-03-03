cmake_minimum_required(VERSION 3.10)
project(mavix-utils-v1 CXX)

# message( "MAVIX::UTILS v1")
# message( "-----------------------------")
# message(STATUS "MAVIX::UTILS v1 LIB DIR:${CMAKE_CURRENT_SOURCE_DIR}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(ABSL_PROPAGATE_CXX_STD ON)

option(LIB_MAVIX_UTILS_BUILD_DOC "Build documentation" OFF)
option(LIB_MAVIX_UTILS_SANITIZE_ADDRESS "SANITIZE ADDRESS" OFF)
option(LIB_MAVIX_UTILS_USE_CATCH ON)
option(LIB_MAVIX_UTILS_USE_LIB ON)
option(LIB_MAVIX_UTILS_USE_TEST OFF)

# Add ASAN
if(LIB_MAVIX_UTILS_SANITIZE_ADDRESS)
    set(SANITIZE_ADDRESS ON)
endif(LIB_MAVIX_UTILS_SANITIZE_ADDRESS)

#  make ourself sanitize and sane well.
if(NOT Sanitizers_FOUND)
 find_package(Sanitizers)
endif()

# Add Doxygen
if(LIB_MAVIX_UTILS_BUILD_DOC)
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
        add_custom_target( doc_mavix_utils ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/generated/docs-cached/
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(LIB_MAVIX_UTILS_BUILD_DOC)


# Add dependencies source code

# Main headers and sources
file(GLOB_RECURSE SOURCES_MAVIX_UTILS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s
)

add_library(${PROJECT_NAME} ${SOURCES_MAVIX_UTILS} )
target_link_libraries(${PROJECT_NAME} 
    nvm-core
    zlib 
    lz4
    mavix-core-v1
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
)


add_executable(${PROJECT_NAME}-test main.cc)
if(Sanitizers_FOUND AND LIB_MAVIX_UTILS_SANITIZE_ADDRESS)
    add_sanitizers("${PROJECT_NAME}-test")
    message(STATUS "${PROJECT_NAME}-test : ASAN OK")
endif()

target_link_libraries(${PROJECT_NAME}-test PRIVATE  ${PROJECT_NAME} )
target_include_directories("${PROJECT_NAME}-test"
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

if(LIB_MAVIX_UTILS_USE_TEST)
message(STATUS "${PROJECT_NAME} unit test : OK")
    add_subdirectory(tests)
endif()       

message(STATUS "${PROJECT_NAME} : OK")
set(mavix-utils-v1_FOUND  ON)