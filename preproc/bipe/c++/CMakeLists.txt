cmake_minimum_required(VERSION 3.14)
project(bipe_app VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# Build Options
# ==============================================================================
option(BUILD_STATIC "Build with full static linking" OFF)

if(BUILD_STATIC)
    # Force CMake to look for static versions of Boost and system libraries
    set(Boost_USE_STATIC_LIBS ON)
    if(NOT MSVC)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ${CMAKE_FIND_LIBRARY_SUFFIXES})
    endif()
endif()

# ==============================================================================
# Sources & Executable
# ==============================================================================
# Gather Application Source Files (Main, ParserDimacs, BManager, etc.)
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "src/*.cpp")

# Define Executable (Named 'bipe' to match your old Makefile)
add_executable(bipe ${SRCS})

# ==============================================================================
# Includes
# ==============================================================================
# Allow c++/src files to include local headers, root bipe headers, and glucose
target_include_directories(bipe PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
	"${CMAKE_CURRENT_SOURCE_DIR}/../"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/glucose-3.0"
)

# ==============================================================================
# Dependencies & Linking
# ==============================================================================
# 1. Boost Program Options
find_package(Boost REQUIRED COMPONENTS program_options)

# 2. ZLIB (Required by bipe/glucose, especially in static builds)
find_package(ZLIB REQUIRED)

# 3. Find the custom 'bipe' library built in the root directory
find_library(BIPE_LIB 
    NAMES bipe libbipe
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../build"
    NO_DEFAULT_PATH
    REQUIRED
)

# Link everything together
# Note: Glucose is already fully baked into BIPE_LIB, so we don't need it here!
target_link_libraries(bipe PRIVATE 
    ${BIPE_LIB}
    Boost::program_options
    ZLIB::ZLIB
)

# ==============================================================================
# Compilation Flags
# ==============================================================================
if(BUILD_STATIC)
    target_compile_definitions(bipe PRIVATE NDEBUG)
    if(NOT MSVC)
        target_link_options(bipe PRIVATE -static)
    endif()
else()
    if(NOT MSVC)
        target_compile_options(bipe PRIVATE -O3)
    endif()
endif()