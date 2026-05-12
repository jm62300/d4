cmake_minimum_required(VERSION 3.14)
project(bipe VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable Position Independent Code (PIC)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Added CONFIGURE_DEPENDS so CMake knows to auto-rebuild if you add new files to the folder.
file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS "./src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "./src/*.hpp")

add_library(bipe ${SRCS})
add_library(bipe::bipe ALIAS bipe) 

# ZLIB
find_package(ZLIB REQUIRED)
target_link_libraries(bipe PUBLIC ZLIB::ZLIB)

# Glucose Integration
add_subdirectory(3rdParty/glucose-3.0)
target_link_libraries(bipe PUBLIC glucose)

# CaDiCaL Integration
add_subdirectory(3rdParty/cadical)
target_link_libraries(bipe PUBLIC cadical)

# Include Directories
target_include_directories(bipe PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/${CMAKE_PROJECT_NAME}>
)

# Compiler-Specific Flags
if(MSVC)
    target_compile_options(bipe PRIVATE /W3)
    target_link_libraries(bipe PRIVATE psapi)
else()
    target_compile_options(bipe PRIVATE 
        -Wall 
        -Wno-sign-compare 
        -Wno-parentheses 
        -Wno-format
    )
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(bipe PRIVATE 
            -Wno-class-memaccess 
            -Wno-misleading-indentation 
            -Wno-literal-suffix
        )
    endif()
endif()

# Installation
install(TARGETS bipe EXPORT bipeTargets DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/${CMAKE_PROJECT_NAME})