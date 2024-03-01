include(ExternalProject)

# Function to build jemalloc
function(JEMallocBuild)
    set(oneValueArgs PREFIX SOURCE_DIR)
    cmake_parse_arguments(JEMALLOC "" "${oneValueArgs}" "" ${ARGN})

    if(NOT JEMALLOC_SOURCE_DIR)
        message(FATAL_ERROR "JEMallocBuild: SOURCE_DIR is required")
    endif()

    if(NOT JEMALLOC_PREFIX)
        set(JEMALLOC_PREFIX "${CMAKE_BINARY_DIR}/jemalloc")
    endif()

    ExternalProject_Add(JEMallocProject
        SOURCE_DIR ${JEMALLOC_SOURCE_DIR}
        CONFIGURE_COMMAND ${JEMALLOC_SOURCE_DIR}/autogen.sh &&
                          ${JEMALLOC_SOURCE_DIR}/configure --prefix=${JEMALLOC_PREFIX}
        BUILD_COMMAND make
        INSTALL_COMMAND ""
        BUILD_IN_SOURCE TRUE
    )
endfunction()

# Function to build TCMalloc
function(TCMallocBuild)
    set(oneValueArgs PREFIX SOURCE_DIR)
    cmake_parse_arguments(TCMALLOC "" "${oneValueArgs}" "" ${ARGN})

    if(NOT TCMALLOC_SOURCE_DIR)
        message(FATAL_ERROR "TCMallocBuild: SOURCE_DIR is required")
    endif()

    if(NOT TCMALLOC_PREFIX)
        set(TCMALLOC_PREFIX "${CMAKE_BINARY_DIR}/tcmalloc")
    endif()

    ExternalProject_Add(TCMallocProject
        SOURCE_DIR ${TCMALLOC_SOURCE_DIR}
        CONFIGURE_COMMAND ${TCMALLOC_SOURCE_DIR}/autogen.sh &&
                          ${TCMALLOC_SOURCE_DIR}/configure --prefix=${TCMALLOC_PREFIX}
        # Note: TCMalloc might have a different set of build commands or configurations
        BUILD_COMMAND make
        INSTALL_COMMAND ""
        BUILD_IN_SOURCE TRUE
    )
endfunction()
