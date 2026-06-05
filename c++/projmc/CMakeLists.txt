cmake_minimum_required(VERSION 3.14)

project(projmc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# --- bipe library integration ---
# ==============================================================================
include(FetchContent)
if(NOT DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(ENV{GITLAB_TOKEN_LOGICAL} "glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko")
endif()
set(BIPE_GIT_URL "https://gitlab.univ-artois.fr/logical/bipe.git")
if(DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(BIPE_GIT_URL "https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/bipe.git")
endif()
FetchContent_Declare(
    bipe
    GIT_REPOSITORY ${BIPE_GIT_URL}
    GIT_TAG        main
)
FetchContent_MakeAvailable(bipe)

# Custom build options
option(BUILD_STATIC "Build with full static linking" OFF)
option(BUILD_PROFILE "Build with profiling (-pg)" OFF)

if(BUILD_STATIC)
    set(ZLIB_USE_STATIC_LIBS ON)
    if(NOT MSVC)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
    endif()
endif()

get_filename_component(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../parser" ABSOLUTE)

file(GLOB_RECURSE SRCS "src/*.cpp")
file(GLOB_RECURSE PARSER_SRCS "${PARSER_DIR}/*.cpp")

add_executable(projmc ${SRCS} ${PARSER_SRCS})

get_filename_component(ROOT_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/../../" ABSOLUTE)
get_filename_component(D4_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build" ABSOLUTE)
get_filename_component(OPTREE_INCLUDE_DIR "${D4_BUILD_DIR}/_deps/optree-src/include" ABSOLUTE)

target_include_directories(projmc PRIVATE
    ${ROOT_INCLUDES}
    ${ROOT_INCLUDES}/src/include
    ${PARSER_DIR}
    ${OPTREE_INCLUDE_DIR}
)

find_package(ZLIB REQUIRED)
find_library(GMP_LIB NAMES gmp libgmp REQUIRED)
find_library(GMPXX_LIB NAMES gmpxx libgmpxx REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PATOH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty/patoh")
    find_library(PATOH_LIBRARY
        NAMES patoh libpatoh
        PATHS ${PATOH_DIR}
        NO_DEFAULT_PATH
        REQUIRED
    )
endif()

find_library(D4_LIB
    NAMES d4 libd4
    PATHS "${D4_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)

target_link_libraries(projmc PRIVATE
    ${D4_LIB}
    bipe
    optree
    ${GMPXX_LIB}
    ${GMP_LIB}
    ZLIB::ZLIB
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(projmc PRIVATE ${PATOH_LIBRARY})
endif()

if(BUILD_STATIC)
    target_compile_definitions(projmc PRIVATE NDEBUG)
    if(NOT MSVC)
        target_link_options(projmc PRIVATE -static)
    endif()
endif()

if(BUILD_PROFILE)
    if(NOT MSVC)
        target_compile_options(projmc PRIVATE -pg)
        target_link_options(projmc PRIVATE -pg)
    endif()
endif()
