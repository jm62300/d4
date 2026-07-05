cmake_minimum_required(VERSION 3.14)

# Set the project name and version
project(D4 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fix: Specify the C++ standard correctly
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (NOT DEFINED BUILD_MODE)
    if (CMAKE_BUILD_TYPE MATCHES "Debug" OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        set(BUILD_MODE 1)
    else ()
        set(BUILD_MODE 0)
    endif ()
endif ()

if (${BUILD_MODE} EQUAL 2)
    # This tells CMake: "When searching for libraries, look for .a BEFORE .so"
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ${CMAKE_FIND_LIBRARY_SUFFIXES})
endif()

# --- Competition mode (BUILD_MODE 4): release + LTO ---
# Benchmarking d4 on this workload (MC2025 track1; AVX2 CPU, no AVX-512) showed:
#   LTO        ~ +6%   (the only real win)
#   -march=native ~ 0  (d4 is pointer-chasing + GMP, not vectorizable)
#   PGO        ~ 0     (actually slightly slower than plain LTO)
# So competition mode = standard release + LTO only. We deliberately do NOT add
# -march=native on the d4/demo side (no benefit, and it would make the binary
# CPU-specific). glucose/flowCutter keep their own -march=native (their release
# default) via their CMakeLists.
# Set GLOBALLY here, BEFORE add_subdirectory(), so glucose/flowCutter/htd inherit
# -flto. -ffat-lto-objects keeps libd4.a linkable by demos that do not opt in.
if (${BUILD_MODE} EQUAL 4 AND NOT MSVC)
    message(STATUS "Build D4 : Mode COMPETITION (4) - -O3 -flto (LTO)")
    add_compile_options(-O3 -flto=auto -ffat-lto-objects)
    add_link_options(-flto=auto)
endif()


# --- 1. Gather Sources ---
file(GLOB_RECURSE SRCS "./src/*.cpp")
file(GLOB_RECURSE HEADERS "./src/*.hpp")

# --- 2. Dependencies ---
    if(CMAKE_PROJECT_NAME STREQUAL "D4")
        set(ZLIB_USE_STATIC_LIBS ON)
    endif()
set(ZLIB_ROOT /usr)

find_package(ZLIB REQUIRED)
find_library(GMP_LIBRARY NAMES gmp libgmp REQUIRED)
find_path(GMP_INCLUDE_DIR NAMES gmp.h REQUIRED)

# GMPXX is usually only explicit on Linux/macOS
if(NOT MSVC)
    find_library(GMPXX_LIBRARY NAMES gmpxx libgmpxx REQUIRED)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "Linux detected: Enabling Patoh support.")
    
    set(PATOH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/patoh")

    find_path(PATOH_INCLUDE_DIR 
        NAMES patoh.h 
        PATHS ${PATOH_DIR} 
        NO_DEFAULT_PATH 
        REQUIRED
    )

    find_library(PATOH_LIBRARY 
        NAMES patoh libpatoh 
        PATHS ${PATOH_DIR} 
        NO_DEFAULT_PATH 
        REQUIRED
    )
else()
    message(STATUS "Non-Linux OS detected: Disabling Patoh support.")
endif()

# --- 3. Subdirectories ---
add_subdirectory(3rdParty/glucose-3.0)
add_subdirectory(3rdParty/flowCutter)

# htd's own CMake relies on write_compiler_detection_header (removed in
# CMake >= 3.19), so its sources are compiled here directly. The headers htd
# normally generates at configure time live in 3rdParty/htdGlue.
file(GLOB HTD_SOURCES "3rdParty/htd/src/htd/*.cpp")
add_library(htd OBJECT ${HTD_SOURCES} "3rdParty/htdGlue/AssemblyInfo.cpp")
target_include_directories(htd PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/htdGlue/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/htd/include"
)
target_compile_options(htd PRIVATE -w)
target_compile_definitions(htd PRIVATE NDEBUG)

# ==============================================================================
# --- optree library integration ---
# ==============================================================================
set(FETCHCONTENT_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty" CACHE PATH "Directory for FetchContent" FORCE)
include(FetchContent)
set(OPTREE_GIT_URL "https://gitlab.univ-artois.fr/logical/optree.git")
if(DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(OPTREE_GIT_URL "https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/optree.git")
endif()
FetchContent_Declare(
    optree
    GIT_REPOSITORY https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/optree.git
    GIT_TAG main
)
FetchContent_MakeAvailable(optree)




# --- 5. BUILD THE MONOLITHIC LIBRARY ---
# By passing $<TARGET_OBJECTS:...>, CMake extracts the raw compiled files 
# from glucose and flowcutter and bundles them directly into libd4.a
add_library(d4 STATIC
    ${SRCS}
    $<TARGET_OBJECTS:glucose>
    $<TARGET_OBJECTS:flowCutter>
    $<TARGET_OBJECTS:htd>
)
add_library(d4::d4 ALIAS d4)

# --- 6. Includes & Linking ---
target_include_directories(d4 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/htdGlue/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/htd/include>
    ${GMP_INCLUDE_DIR}
)

# WIN : D4 find glucose bip before glucose
target_include_directories(d4 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0") 

# Link dependencies
if (MSVC)
    target_link_libraries(d4 PUBLIC ZLIB::ZLIB ${GMP_LIBRARY} optree)
else ()
    target_link_libraries(d4 PUBLIC ZLIB::ZLIB ${GMPXX_LIBRARY} ${GMP_LIBRARY} optree)
endif ()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_include_directories(d4 PUBLIC ${PATOH_INCLUDE_DIR})
    target_link_libraries(d4 PUBLIC ${PATOH_LIBRARY})
    
    # This creates a #define USE_PATOH in your C++ code!
    target_compile_definitions(d4 PUBLIC USE_PATOH)
endif()

# --- 7. Compiler Options ---
if (MSVC)
    # NOMINMAX must be defined before any windows.h inclusion (including via gmpxx.h)
    # so we set it globally for all MSVC compilation units.
    add_compile_definitions(NOMINMAX)
    target_compile_options(d4 PUBLIC /W3 /wd4477 /wd4455 /wd4018 /wd4706)
else ()
    target_compile_options(d4 PUBLIC -Wall -Wno-strict-aliasing -Wno-format -Wno-sign-compare -Wno-parentheses -Wno-restrict)
    # GCC specific flags (Clang will crash if it sees these)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(d4 PUBLIC -Wno-class-memaccess -Wno-misleading-indentation -Wno-literal-suffix)
    endif()
endif ()

# --- 8. Build Modes ---
if (NOT DEFINED BUILD_MODE)
    if (CMAKE_BUILD_TYPE MATCHES "Debug" OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        set(BUILD_MODE 1)
    else ()
        set(BUILD_MODE 0)
    endif ()
endif ()

if (${BUILD_MODE} EQUAL 1)
    message(STATUS "Build D4 : Mode DEBUG (1)")
    if (MSVC)
        target_compile_options(d4 PUBLIC /Zi /Od /U NDEBUG)
    else ()
        target_compile_options(d4 PUBLIC -g -O0 -UNDEBUG) # changed to -O0 for actual debug
    endif ()
    set(SUFFIX_DIR "-debug")

elseif (${BUILD_MODE} EQUAL 2)
    message(STATUS "Build D4 : Mode STATIC (2)")
    target_compile_definitions(d4 PUBLIC NDEBUG)
    if (MSVC)
        target_compile_options(d4 PUBLIC $<$<NOT:$<CONFIG:Debug>>:/O2> /MT)
    else ()
        target_compile_options(d4 PUBLIC -O3 -march=native)
        target_link_options(d4 PUBLIC -static)
    endif ()
    set(SUFFIX_DIR "-static")

elseif (${BUILD_MODE} EQUAL 4)
    message(STATUS "Build D4 : Mode COMPETITION (4)")
    target_compile_definitions(d4 PUBLIC NDEBUG)
    # Optimization/LTO flags are applied globally above (add_compile_options),
    # so glucose/flowCutter/htd pick them up as well.
    set(SUFFIX_DIR "-competition")

elseif (${BUILD_MODE} EQUAL 3)
    message(STATUS "Build D4 : Mode PROFILING (3)")
    target_compile_definitions(d4 PUBLIC NDEBUG)
    if (MSVC)
        target_compile_options(d4 PUBLIC /Zi $<$<NOT:$<CONFIG:Debug>>:/O2>)
        target_link_options(d4 PRIVATE /PROFILE)
    else ()
        target_compile_options(d4 PUBLIC -pg -g -O3)
        target_link_options(d4 PRIVATE -pg -g)
    endif ()
    set(SUFFIX_DIR "-profil")

else ()
    message(STATUS "Build D4 : Mode STANDARD/RELEASE (0)")
    target_compile_definitions(d4 PUBLIC NDEBUG)
    if (MSVC)
        target_compile_options(d4 PUBLIC $<$<NOT:$<CONFIG:Debug>>:/O2>)
    else ()
        target_compile_options(d4 PUBLIC -O3)
    endif ()
    set(SUFFIX_DIR "-standard")
endif ()

# --- 9. Installation ---
install(TARGETS d4 DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/${CMAKE_PROJECT_NAME}${SUFFIX_DIR})

# Optional: Keep the cleanup target if you use it manually
add_custom_target(clean-cmake-files COMMAND ${CMAKE_COMMAND} -P clean-all.cmake)
