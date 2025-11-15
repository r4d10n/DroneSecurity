cmake_minimum_required(VERSION 3.10)
project(droneid_receiver C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Version information
set(DRONEID_VERSION_MAJOR 1)
set(DRONEID_VERSION_MINOR 0)
set(DRONEID_VERSION_PATCH 0)

# Build options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(ENABLE_NEON "Enable ARM NEON optimizations" ON)
option(ENABLE_NE10 "Use Ne10 library for FFT" ON)
option(ENABLE_LIBIIO "Build with libiio support for PlutoSDR" ON)
option(BUILD_TESTS "Build test programs" ON)
option(ENABLE_EXAMPLES "Build example programs" ON)

# Detect architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(IS_ARM TRUE)
else()
    set(IS_ARM FALSE)
endif()

# Compiler flags
if(IS_ARM AND ENABLE_NEON)
    # ARM Cortex-A9 with NEON
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard")
    add_definitions(-DHAVE_NEON=1)
    message(STATUS "ARM NEON optimizations enabled")
endif()

# Optimization flags
set(CMAKE_C_FLAGS_RELEASE "-O3 -ffast-math -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g -Wall -Wextra")

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files
set(COMMON_SOURCES
    src/common.c
    src/dsp_neon.c
)

# Find dependencies
find_package(Threads REQUIRED)
find_library(LIBM m)

# Ne10 library (optional)
if(ENABLE_NE10)
    find_library(NE10_LIBRARY NE10 PATHS /usr/lib /usr/local/lib)
    find_path(NE10_INCLUDE_DIR NE10.h PATHS /usr/include /usr/local/include)

    if(NE10_LIBRARY AND NE10_INCLUDE_DIR)
        message(STATUS "Found Ne10: ${NE10_LIBRARY}")
        include_directories(${NE10_INCLUDE_DIR})
        add_definitions(-DHAVE_NE10=1)
        set(EXTRA_LIBS ${EXTRA_LIBS} ${NE10_LIBRARY})
    else()
        message(WARNING "Ne10 not found, FFT will use fallback implementation")
    endif()
endif()

# libiio (optional, for PlutoSDR)
if(ENABLE_LIBIIO)
    find_library(IIO_LIBRARY iio PATHS /usr/lib /usr/local/lib)
    find_path(IIO_INCLUDE_DIR iio.h PATHS /usr/include /usr/local/include)

    if(IIO_LIBRARY AND IIO_INCLUDE_DIR)
        message(STATUS "Found libiio: ${IIO_LIBRARY}")
        include_directories(${IIO_INCLUDE_DIR})
        add_definitions(-DHAVE_LIBIIO=1)
        set(EXTRA_LIBS ${EXTRA_LIBS} ${IIO_LIBRARY})
    else()
        message(WARNING "libiio not found, PlutoSDR support disabled")
    endif()
endif()

# Main library
add_library(droneid ${COMMON_SOURCES})
target_link_libraries(droneid ${EXTRA_LIBS} ${LIBM} ${CMAKE_THREAD_LIBS_INIT})

# Installation
install(TARGETS droneid
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include/droneid
    FILES_MATCHING PATTERN "*.h"
)

# Examples (if enabled)
if(ENABLE_EXAMPLES)
    add_subdirectory(examples EXCLUDE_FROM_ALL)
endif()

# Tests (if enabled)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests EXCLUDE_FROM_ALL)
endif()

# Documentation
configure_file(
    ${CMAKE_SOURCE_DIR}/docs/ARCHITECTURE.md
    ${CMAKE_BINARY_DIR}/docs/ARCHITECTURE.md
    COPYONLY
)

# Print summary
message(STATUS "")
message(STATUS "DroneSecurity C Implementation - Configuration Summary")
message(STATUS "======================================================")
message(STATUS "  Version:          ${DRONEID_VERSION_MAJOR}.${DRONEID_VERSION_MINOR}.${DRONEID_VERSION_PATCH}")
message(STATUS "  Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "  Architecture:     ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  C Compiler:       ${CMAKE_C_COMPILER}")
message(STATUS "  C Flags:          ${CMAKE_C_FLAGS}")
message(STATUS "")
message(STATUS "  Options:")
message(STATUS "    NEON:           ${ENABLE_NEON} (detected: ${IS_ARM})")
message(STATUS "    Ne10:           ${ENABLE_NE10}")
message(STATUS "    libiio:         ${ENABLE_LIBIIO}")
message(STATUS "    Tests:          ${BUILD_TESTS}")
message(STATUS "    Examples:       ${ENABLE_EXAMPLES}")
message(STATUS "")
message(STATUS "  Install prefix:   ${CMAKE_INSTALL_PREFIX}")
message(STATUS "======================================================")
message(STATUS "")
