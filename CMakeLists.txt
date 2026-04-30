cmake_minimum_required(VERSION 3.14)

# Set the project name and version
project(D4 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Fix: Specify the C++ standard correctly
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- 1. Gather Sources ---
file(GLOB_RECURSE SRCS "./src/*.cpp")
file(GLOB_RECURSE HEADERS "./src/*.hpp")

# --- 2. Dependencies ---
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_library(GMP_LIBRARY NAMES gmp libgmp REQUIRED)
find_path(GMP_INCLUDE_DIR NAMES gmp.h REQUIRED)

# GMPXX is usually only explicit on Linux/macOS
if(NOT MSVC)
    find_library(GMPXX_LIBRARY NAMES gmpxx libgmpxx REQUIRED)
endif()

# --- 3. Subdirectories ---
add_subdirectory(3rdParty/glucose-3.0)
add_subdirectory(3rdParty/flowCutter)

# --- 4. Python Integration ---
option(BUILD_PYTHON_WRAPPER "Build wrapper Python" OFF)
if (SKBUILD OR BUILD_PYTHON_WRAPPER)
    add_subdirectory(python)
endif ()

# --- 5. BUILD THE MONOLITHIC LIBRARY ---
# By passing $<TARGET_OBJECTS:...>, CMake extracts the raw compiled files 
# from glucose and flowcutter and bundles them directly into libd4.a
add_library(d4 STATIC 
    ${SRCS} 
    $<TARGET_OBJECTS:glucose> 
    $<TARGET_OBJECTS:flowCutter>
)
add_library(d4::d4 ALIAS d4)

# --- 6. Includes & Linking ---
target_include_directories(d4 PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${GMP_INCLUDE_DIR}
)

if(Boost_INCLUDE_DIRS)
    target_include_directories(d4 PUBLIC ${Boost_INCLUDE_DIRS})
endif()

# WIN : D4 find glucose bip before glucose
target_include_directories(d4 PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0") 

# Link dependencies (Note: glucose and flowcutter are now natively inside d4, so we don't link them here!)
if (MSVC)
    target_link_libraries(d4 PUBLIC ZLIB::ZLIB ${GMP_LIBRARY})
else ()
    target_link_libraries(d4 PUBLIC ZLIB::ZLIB ${GMPXX_LIBRARY} ${GMP_LIBRARY})
endif ()

# --- 7. Compiler Options ---
if (MSVC)
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
        target_compile_options(d4 PUBLIC -O2)
        target_link_options(d4 PUBLIC -static)
    endif ()
    set(SUFFIX_DIR "-static")

elseif (${BUILD_MODE} EQUAL 3)
    message(STATUS "Build D4 : Mode PROFILING (3)")
    target_compile_definitions(d4 PUBLIC NDEBUG)
    if (MSVC)
        target_compile_options(d4 PUBLIC /Zi $<$<NOT:$<CONFIG:Debug>>:/O2>)
        target_link_options(d4 PRIVATE /PROFILE)
    else ()
        target_compile_options(d4 PUBLIC -p -g -O3)
        target_link_options(d4 PRIVATE -pg)
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