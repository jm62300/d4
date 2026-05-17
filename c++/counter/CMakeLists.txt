cmake_minimum_required(VERSION 3.14)

# Set the project name
project(counter LANGUAGES CXX)

# Specify the C++ standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- MOVE OPTIONS HERE ---
# Custom Build Options 
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


# Get the absolute path to the parser directory safely
get_filename_component(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../parser" ABSOLUTE)

# Gather Sources
file(GLOB_RECURSE SRCS "src/*.cpp")
# Grab the parser source files (ParserDimacs.cpp)
file(GLOB_RECURSE PARSER_SRCS "${PARSER_DIR}/*.cpp") 

# Define the Executable (Now includes the parser sources!)
add_executable(counter ${SRCS} ${PARSER_SRCS})

# Include Directories
get_filename_component(ROOT_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)
get_filename_component(BIPE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../preproc/bipe/" ABSOLUTE)

target_include_directories(counter PRIVATE 
    ${ROOT_INCLUDES}
    ${ROOT_INCLUDES}/src/include
    ${PARSER_DIR}
    ${BIPE_INCLUDE_DIR}
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
get_filename_component(D4_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build" ABSOLUTE)
find_library(D4_LIB 
    NAMES d4 libd4
    PATHS "${D4_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)

# Bipe Library
get_filename_component(BIPE_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../preproc/bipe/build/" ABSOLUTE)
find_library(BIPE_LIB 
    NAMES bipe libbipe
    PATHS "${BIPE_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)


# Link Dependencies
target_link_libraries(counter PRIVATE 
    ${D4_LIB}
    ${BIPE_LIB}
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
        target_compile_options(counter PRIVATE -pg)
        target_link_options(counter PRIVATE -pg)
    endif()
endif()