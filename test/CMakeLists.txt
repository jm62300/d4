cmake_minimum_required(VERSION 3.14)
project(test_api LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

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



# Set FetchContent base directory to the shared 3rdParty directory
get_filename_component(SHARED_3RDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty" ABSOLUTE)
set(FETCHCONTENT_BASE_DIR "${SHARED_3RDPARTY_DIR}" CACHE PATH "Directory for FetchContent" FORCE)

# Fetch optree (header-only library)
include(FetchContent)
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

# Integrated BiPe Library
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


# Build CaDiCaL from 3rdParty
if (NOT TARGET cadical)
    add_subdirectory("${ROOT_DIR}/3rdParty/cadical" "${CMAKE_CURRENT_BINARY_DIR}/cadical_build")
endif()

# Define the test executable
add_executable(test_api 
    test_api.cpp 
    "${ROOT_DIR}/api/solver/Solver.cpp" 
    "${ROOT_DIR}/c++/parser/ParserDimacs.cpp"
    "${ROOT_DIR}/c++/parser/ParserCircuit.cpp"
)

add_executable(test_crash_19 
    test_crash_19.cpp 
    "${ROOT_DIR}/api/solver/Solver.cpp" 
    "${ROOT_DIR}/c++/parser/ParserDimacs.cpp"
    "${ROOT_DIR}/c++/parser/ParserCircuit.cpp"
)

add_executable(test_circuit_api 
    test_circuit_api.cpp 
    "${ROOT_DIR}/api/solver/Solver.cpp" 
    "${ROOT_DIR}/c++/parser/ParserDimacs.cpp"
    "${ROOT_DIR}/c++/parser/ParserCircuit.cpp"
)

add_executable(test_wmc_api 
    test_wmc_api.cpp 
    "${ROOT_DIR}/api/solver/Solver.cpp" 
    "${ROOT_DIR}/c++/parser/ParserDimacs.cpp"
    "${ROOT_DIR}/c++/parser/ParserCircuit.cpp"
)


# Include Directories
target_include_directories(test_api PRIVATE
    ${ROOT_DIR}
    ${GMP_INCLUDE_DIR}
)
target_include_directories(test_crash_19 PRIVATE
    ${ROOT_DIR}
    ${GMP_INCLUDE_DIR}
)
target_include_directories(test_circuit_api PRIVATE
    ${ROOT_DIR}
    ${GMP_INCLUDE_DIR}
)
target_include_directories(test_wmc_api PRIVATE
    ${ROOT_DIR}
    ${GMP_INCLUDE_DIR}
)



# Link everything
if (MSVC)
    target_link_libraries(test_api PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMP_LIBRARY}
    )
    target_link_libraries(test_crash_19 PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMP_LIBRARY}
    )
    target_link_libraries(test_circuit_api PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMP_LIBRARY}
    )
    target_link_libraries(test_wmc_api PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMP_LIBRARY}
    )

else()
    target_link_libraries(test_api PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMPXX_LIBRARY}
        ${GMP_LIBRARY}
    )
    target_link_libraries(test_crash_19 PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMPXX_LIBRARY}
        ${GMP_LIBRARY}
    )
    target_link_libraries(test_circuit_api PRIVATE 
        ${D4_LIB}
        bipe
        cadical
        optree
        ZLIB::ZLIB
        ${GMPXX_LIBRARY}
        ${GMP_LIBRARY}
    )
    target_link_libraries(test_wmc_api PRIVATE 
        ${D4_LIB}
        bipe
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
    target_link_libraries(test_crash_19 PRIVATE ${PATOH_LIBRARY})
    target_compile_definitions(test_crash_19 PRIVATE USE_PATOH)
    target_link_libraries(test_circuit_api PRIVATE ${PATOH_LIBRARY})
    target_compile_definitions(test_circuit_api PRIVATE USE_PATOH)
    target_link_libraries(test_wmc_api PRIVATE ${PATOH_LIBRARY})
    target_compile_definitions(test_wmc_api PRIVATE USE_PATOH)

endif()

