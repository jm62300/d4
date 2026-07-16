cmake_minimum_required(VERSION 3.14)

# Set the project name
project(compiler LANGUAGES CXX)

# Disable install and export commands to prevent static build export validation errors on fetched dependencies
macro(install)
endmacro()
macro(export)
endmacro()

# Specify the C++ standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Custom Build Options ---
# Must be set BEFORE FetchContent_MakeAvailable(bipe): bipe's own CMakeLists.txt
# calls find_package(ZLIB) internally, which caches ZLIB_LIBRARY_RELEASE on
# first search. If that happens before CMAKE_FIND_LIBRARY_SUFFIXES is
# restricted to ".a", it caches the .so path and our later find_package(ZLIB)
# below just reuses it, silently defeating BUILD_STATIC.
option(BUILD_STATIC "Build with full static linking" OFF)
option(BUILD_PROFILE "Build with profiling (-p/-pg)" OFF)

if(BUILD_STATIC)
    # Tell CMake's find_package to strictly look for static libraries
    set(ZLIB_USE_STATIC_LIBS ON)

    # Tell CMake's find_library to look for .a files instead of .so files
    if(NOT MSVC)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
    endif()
endif()
# -------------------------

# ==============================================================================
# --- bipe library integration (renaming its cadical to bipe_cadical) ---
# ==============================================================================
get_filename_component(SHARED_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty" ABSOLUTE)
set(FETCHCONTENT_BASE_DIR "${SHARED_3RDPARTY_DIR}" CACHE PATH "Directory for FetchContent" FORCE)
include(FetchContent)
set(BIPE_GIT_URL "git@github.com:jm62300/bipe.git")
FetchContent_Declare(
    bipe
    GIT_REPOSITORY ${BIPE_GIT_URL}
    GIT_TAG        v1.0.0
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


# Get the absolute path to the parser directory safely
get_filename_component(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../parser" ABSOLUTE)

# Gather Sources
file(GLOB_RECURSE SRCS "src/*.cpp")
# Grab the parser source files (ParserDimacs.cpp)
file(GLOB_RECURSE PARSER_SRCS "${PARSER_DIR}/*.cpp") 

# Define the Executable (Now includes the parser sources!)
add_executable(compiler ${SRCS} ${PARSER_SRCS})

# Include Directories
get_filename_component(ROOT_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)
get_filename_component(D4_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build" ABSOLUTE)
get_filename_component(SHARED_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty" ABSOLUTE)
get_filename_component(OPTREE_INCLUDE_DIR "${SHARED_3RDPARTY_DIR}/optree-src/include" ABSOLUTE)

target_include_directories(compiler PRIVATE 
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
    # Adjust this path to point accurately to the 3rdParty/patoh folder from the compiler directory
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
target_link_libraries(compiler PRIVATE 
    ${D4_LIB}
    bipe
    optree
    ${GMPXX_LIB}
    ${GMP_LIB}
    ZLIB::ZLIB
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(compiler PRIVATE ${PATOH_LIBRARY})
endif()

# Apply Target-Specific Flags
if(BUILD_STATIC)
    target_compile_definitions(compiler PRIVATE NDEBUG)
    if(NOT MSVC)
        target_link_options(compiler PRIVATE -static)
    endif()
endif()

if(BUILD_PROFILE)
    if(NOT MSVC)
        target_compile_options(compiler PRIVATE -pg)
        target_link_options(compiler PRIVATE -pg)
    endif()
endif()