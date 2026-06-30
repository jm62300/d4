cmake_minimum_required(VERSION 3.14)

# Set the project name
project(counter LANGUAGES CXX)

# Specify the C++ standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Custom Build Options ---
# Must be set before FetchContent_MakeAvailable(bipe) below, since bipe's own
# CMakeLists.txt calls find_package(ZLIB) and caches whatever it finds first;
# if the static hints aren't in place yet, bipe locks in the .so and a later
# -static link fails trying to statically link that dynamic object.
option(BUILD_STATIC "Build with full static linking" OFF)
option(BUILD_PROFILE "Build with profiling (-p/-pg)" OFF)
option(BUILD_COMPETITION "Build with -O3 -march=native -flto (compile on target cluster)" OFF)

if(BUILD_STATIC)
    # Tell CMake's find_package to strictly look for static libraries
    set(ZLIB_USE_STATIC_LIBS ON)

    # Tell CMake's find_library to look for .a files instead of .so files
    if(NOT MSVC)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
    endif()
endif()
# -------------------------
set(CMAKE_SKIP_INSTALL_ALL TRUE)

include(FetchContent)

# ==============================================================================
# --- bipe library integration (renaming its cadical to bipe_cadical) ---
# ==============================================================================
get_filename_component(SHARED_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty" ABSOLUTE)
set(FETCHCONTENT_BASE_DIR "${SHARED_3RDPARTY_DIR}" CACHE PATH "Directory for FetchContent" FORCE)
set(BIPE_GIT_URL "https://gitlab.univ-artois.fr/logical/bipe.git")
if(DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(BIPE_GIT_URL "https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/bipe.git")
endif()
FetchContent_Declare(
    bipe
    GIT_REPOSITORY ${BIPE_GIT_URL}
    GIT_TAG        main
)
FetchContent_GetProperties(bipe)
if(NOT bipe_POPULATED)
    FetchContent_Populate(bipe)
    
    # 1. Patch bipe-src/CMakeLists.txt to link against bipe_cadical instead of cadical
    set(BIPE_CMAKELIST "${bipe_SOURCE_DIR}/CMakeLists.txt")
    if(EXISTS "${BIPE_CMAKELIST}")
        file(READ "${BIPE_CMAKELIST}" BIPE_CONTENT)
        string(REPLACE "PUBLIC cadical" "PUBLIC bipe_cadical" BIPE_CONTENT "${BIPE_CONTENT}")
        file(WRITE "${BIPE_CMAKELIST}" "${BIPE_CONTENT}")
    endif()

    # 2. Patch bipe-src/3rdParty/cadical/CMakeLists.txt to rename the target to bipe_cadical
    set(BIPE_CADICAL_CMAKELIST "${bipe_SOURCE_DIR}/3rdParty/cadical/CMakeLists.txt")
    if(EXISTS "${BIPE_CADICAL_CMAKELIST}")
        file(READ "${BIPE_CADICAL_CMAKELIST}" BIPE_CADICAL_CONTENT)
        string(REPLACE "add_library(cadical " "add_library(bipe_cadical " BIPE_CADICAL_CONTENT "${BIPE_CADICAL_CONTENT}")
        string(REPLACE "target_include_directories(cadical " "target_include_directories(bipe_cadical " BIPE_CADICAL_CONTENT "${BIPE_CADICAL_CONTENT}")
        string(REPLACE "target_compile_definitions(cadical " "target_compile_definitions(bipe_cadical " BIPE_CADICAL_CONTENT "${BIPE_CADICAL_CONTENT}")
        string(REPLACE "target_compile_options(cadical " "target_compile_options(bipe_cadical " BIPE_CADICAL_CONTENT "${BIPE_CADICAL_CONTENT}")
        file(WRITE "${BIPE_CADICAL_CMAKELIST}" "${BIPE_CADICAL_CONTENT}")
    endif()

    add_subdirectory(${bipe_SOURCE_DIR} ${bipe_BINARY_DIR})
endif()

# ==============================================================================
# --- isolate cadical for arjun/cryptominisat5/cadiback into CaDiCaLMeel namespace ---
# ==============================================================================
add_definitions(-DCaDiCaL=CaDiCaLMeel)

# ==============================================================================
# --- arjun and dependencies library integration ---
# ==============================================================================
set(MPFR_HELPER_DIR "${CMAKE_BINARY_DIR}/mpfr-helper")
file(MAKE_DIRECTORY "${MPFR_HELPER_DIR}/include")
file(MAKE_DIRECTORY "${MPFR_HELPER_DIR}/lib")
file(MAKE_DIRECTORY "${MPFR_HELPER_DIR}/lib/pkgconfig")

if(NOT EXISTS "${MPFR_HELPER_DIR}/include/mpfr.h")
    message(STATUS "Downloading mpfr.h from official INRIA Gitlab...")
    file(DOWNLOAD 
        "https://gitlab.inria.fr/mpfr/mpfr/-/raw/4.1.0/src/mpfr.h" 
        "${MPFR_HELPER_DIR}/include/mpfr.h"
        STATUS download_status
    )
    list(GET download_status 0 status_code)
    if(NOT status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download mpfr.h: ${download_status}")
    endif()
endif()

if(NOT EXISTS "${MPFR_HELPER_DIR}/lib/libmpfr.so")
    message(STATUS "Creating local libmpfr.so symlink pointing to /usr/lib/x86_64-linux-gnu/libmpfr.so.6...")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink 
        "/usr/lib/x86_64-linux-gnu/libmpfr.so.6" 
        "${MPFR_HELPER_DIR}/lib/libmpfr.so"
    )
endif()

file(WRITE "${MPFR_HELPER_DIR}/lib/pkgconfig/mpfr.pc"
"prefix=${MPFR_HELPER_DIR}
libdir=\${prefix}/lib
includedir=\${prefix}/include

Name: mpfr
Description: C library for multiple-precision floating-point computations (mocked)
Version: 4.1.0
Libs: -L\${libdir} -lmpfr
Cflags: -I\${includedir}
")

set(ENV{PKG_CONFIG_PATH} "${MPFR_HELPER_DIR}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

FetchContent_Declare(
    cadiback
    GIT_REPOSITORY https://github.com/meelgroup/cadiback.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(cadiback)
if(NOT cadiback_POPULATED)
    FetchContent_Populate(cadiback)
    set(CADIBACK_CMAKELIST "${cadiback_SOURCE_DIR}/CMakeLists.txt")
    if(EXISTS "${CADIBACK_CMAKELIST}")
        file(READ "${CADIBACK_CMAKELIST}" CADIBACK_CONTENT)
        string(FIND "${CADIBACK_CONTENT}" "# ---- Install" INSTALL_INDEX)
        if(NOT INSTALL_INDEX EQUAL -1)
            string(SUBSTRING "${CADIBACK_CONTENT}" 0 ${INSTALL_INDEX} CADIBACK_CLEAN)
            file(WRITE "${CADIBACK_CMAKELIST}" "${CADIBACK_CLEAN}")
        endif()
    endif()
    add_subdirectory(${cadiback_SOURCE_DIR} ${cadiback_BINARY_DIR})
endif()

FetchContent_Declare(
    arjun
    GIT_REPOSITORY https://github.com/meelgroup/arjun.git
    GIT_TAG        master
)
FetchContent_GetProperties(arjun)
if(NOT arjun_POPULATED)
    FetchContent_Populate(arjun)
    set(ARJUN_FORMULA_H "${arjun_SOURCE_DIR}/src/formula.h")
    if(EXISTS "${ARJUN_FORMULA_H}")
        file(READ "${ARJUN_FORMULA_H}" ARJUN_FORMULA_CONTENT)
        string(REPLACE "constexpr CL(" "CL(" ARJUN_FORMULA_CONTENT "${ARJUN_FORMULA_CONTENT}")
        file(WRITE "${ARJUN_FORMULA_H}" "${ARJUN_FORMULA_CONTENT}")
    endif()
    add_subdirectory(${arjun_SOURCE_DIR} ${arjun_BINARY_DIR})
endif()

# ==============================================================================
# --- remove namespace isolation for following targets ---
# ==============================================================================
remove_definitions(-DCaDiCaL=CaDiCaLMeel)


# Get the absolute path to the parser directory safely
get_filename_component(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../parser" ABSOLUTE)

# Gather Sources
file(GLOB_RECURSE SRCS "src/*.cpp")
# Grab the parser source files (ParserDimacs.cpp)
file(GLOB_RECURSE PARSER_SRCS "${PARSER_DIR}/*.cpp") 

# Define the Executable (Now includes the parser sources!)
add_executable(counter ${SRCS} ${PARSER_SRCS})
if(TARGET CopyPublicHeaders_arjun)
    add_dependencies(counter CopyPublicHeaders_arjun)
endif()

# Include Directories
get_filename_component(ROOT_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)
get_filename_component(D4_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build" ABSOLUTE)
get_filename_component(SHARED_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty" ABSOLUTE)
get_filename_component(OPTREE_INCLUDE_DIR "${SHARED_3RDPARTY_DIR}/optree-src/include" ABSOLUTE)

target_include_directories(counter PRIVATE 
    ${ROOT_INCLUDES}
    ${ROOT_INCLUDES}/src/include
    ${PARSER_DIR}
    ${OPTREE_INCLUDE_DIR}
)

# Find Dependencies
# ZLIB
find_package(ZLIB REQUIRED)

# GMP & GMPXX
find_library(GMP_LIB NAMES gmp libgmp REQUIRED)
find_library(GMPXX_LIB NAMES gmpxx libgmpxx REQUIRED)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Adjust this path to point accurately to the 3rdParty/patoh folder from the counter directory
    # Based on your previous paths, it should look something like this:
    set(PATOH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty/patoh")

    find_library(PATOH_LIBRARY 
        NAMES patoh libpatoh 
        PATHS ${PATOH_DIR} 
        NO_DEFAULT_PATH 
        REQUIRED
    )
endif()



# D4 Library
find_library(D4_LIB 
    NAMES d4 libd4
    PATHS "${D4_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)



# Link Dependencies
target_link_libraries(counter PRIVATE 
    ${D4_LIB}
    bipe
    arjun
    optree
    ${GMPXX_LIB}
    ${GMP_LIB}
    ZLIB::ZLIB
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(counter PRIVATE ${PATOH_LIBRARY})
endif()

# Apply Target-Specific Flags
if(BUILD_STATIC)
    target_compile_definitions(counter PRIVATE NDEBUG)
    if(NOT MSVC)
        target_link_options(counter PRIVATE -static)
    endif()
endif()

if(BUILD_PROFILE)
    if(NOT MSVC)
        target_compile_options(counter PRIVATE -pg -g)
        target_link_options(counter PRIVATE -pg -g)
    endif()
endif()

if(BUILD_COMPETITION)
    target_compile_definitions(counter PRIVATE NDEBUG)
    if(NOT MSVC)
        # Competition = release + LTO (benchmarks showed -march=native/PGO add
        # nothing for d4). LTO must be on the final link to cross the libd4.a
        # boundary; -ffat-lto-objects keeps linking robust if any object lacks IR.
        target_compile_options(counter PRIVATE -O3 -flto=auto -ffat-lto-objects)
        target_link_options(counter PRIVATE -flto=auto)
    endif()
endif()