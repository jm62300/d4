cmake_minimum_required(VERSION 3.14)
project(test_api LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Root directory of the repository
get_filename_component(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(D4_BUILD_DIR "${ROOT_DIR}/build")

# Find the built d4 static library
find_library(D4_LIB 
    NAMES d4 libd4
    PATHS "${D4_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)

# Find ZLIB
find_package(ZLIB REQUIRED)

# Find GMP & GMPXX
find_library(GMP_LIBRARY NAMES gmp libgmp REQUIRED)
find_path(GMP_INCLUDE_DIR NAMES gmp.h REQUIRED)
if(NOT MSVC)
    find_library(GMPXX_LIBRARY NAMES gmpxx libgmpxx REQUIRED)
endif()

# Find Boost
find_package(Boost REQUIRED)

# Fetch optree (header-only library)
include(FetchContent)
if(NOT DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(ENV{GITLAB_TOKEN_LOGICAL} "glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko")
endif()
set(OPTREE_GIT_URL "https://gitlab.univ-artois.fr/logical/optree.git")
if(DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(OPTREE_GIT_URL "https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/optree.git")
endif()
FetchContent_Declare(
    optree
    GIT_REPOSITORY ${OPTREE_GIT_URL}
    GIT_TAG        main
)
FetchContent_MakeAvailable(optree)

# Build CaDiCaL from 3rdParty
add_subdirectory("${ROOT_DIR}/3rdParty/cadical" "${CMAKE_CURRENT_BINARY_DIR}/cadical_build")

# Define the test executable
add_executable(test_api 
    test_api.cpp 
    "${ROOT_DIR}/api/runner/Counter.cpp" 
    "${ROOT_DIR}/c++/parser/ParserDimacs.cpp"
)

# Include Directories
target_include_directories(test_api PRIVATE
    ${ROOT_DIR}
    ${GMP_INCLUDE_DIR}
)
if(Boost_INCLUDE_DIRS)
    target_include_directories(test_api PRIVATE ${Boost_INCLUDE_DIRS})
endif()

# Link everything
if (MSVC)
    target_link_libraries(test_api PRIVATE 
        ${D4_LIB}
        cadical
        optree
        ZLIB::ZLIB
        ${GMP_LIBRARY}
    )
else()
    target_link_libraries(test_api PRIVATE 
        ${D4_LIB}
        cadical
        optree
        ZLIB::ZLIB
        ${GMPXX_LIBRARY}
        ${GMP_LIBRARY}
    )
endif()

# Handle Patoh (Linux only)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PATOH_DIR "${ROOT_DIR}/3rdParty/patoh")
    find_library(PATOH_LIBRARY 
        NAMES patoh libpatoh 
        PATHS ${PATOH_DIR} 
        NO_DEFAULT_PATH 
        REQUIRED
    )
    target_link_libraries(test_api PRIVATE ${PATOH_LIBRARY})
    target_compile_definitions(test_api PRIVATE USE_PATOH)
endif()
