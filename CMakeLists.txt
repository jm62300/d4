cmake_minimum_required(VERSION 3.5)

# set the project name and version
project(D4 VERSION 0.1)

add_custom_target(clean-cmake-files
        COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_BUILD_TYPE Release)

# specify the C++ standard
set(CMAKE_CXX_STANDARD_REQUIRED 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# source/include files
file(GLOB_RECURSE SOURCES ./src/**.cpp)
file(GLOB_RECURSE INCLUDES ./src/**.hpp)

set(SRCS ${SOURCES})
set(HEADERS ${INCLUDES})

set(Boost_USE_STATIC_LIBS ON)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# --- INTEGRATION PYTHON (nanobind) ---
option(BUILD_PYTHON_WRAPPER "Build wapper Python" OFF)
if (SKBUILD OR BUILD_PYTHON_WRAPPER)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_subdirectory(python)
endif ()


if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif ()

if (NOT DEFINED BUILD_MODE)
    if (CMAKE_BUILD_TYPE MATCHES "Debug" OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        set(BUILD_MODE 1)
    else ()
        set(BUILD_MODE 0)
    endif ()
endif ()

find_package(ZLIB REQUIRED)
find_library(GMP_LIBRARY NAMES gmp libgmp REQUIRED)
find_path(GMP_INCLUDE_DIR NAMES gmp.h REQUIRED)

add_subdirectory(3rdParty/glucose-3.0)
add_subdirectory(3rdParty/flowCutter)

set(libList
        ${Boost_LIBRARIES}
        flowCutter
        glucose
)

# build.
add_library(d4 STATIC ${SRCS} ${HEADERS})
add_library(d4::d4 ALIAS d4)
target_include_directories(d4 BEFORE PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0") # WIN : D4 find glucose bip before glucose
target_include_directories(d4 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/include" ${GMP_INCLUDE_DIR})

target_compile_features(d4 PUBLIC cxx_std_20)

if (MSVC)
    set(wnoOption_MSVC /wd4477 /wd4455 /wd4018 /wd4706)
    target_compile_options(d4 PUBLIC
            /W3
            ${wnoOption_MSVC}
    )
else ()
    set(wnoOption -Wno-strict-aliasing -Wno-class-memaccess -Wno-format -Wno-misleading-indentation -Wno-literal-suffix -Wno-sign-compare -Wno-parentheses -Wno-restrict)
    target_compile_options(d4 PUBLIC
            -Wall
            ${wnoOption}
    )
endif ()

target_include_directories(d4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if (MSVC)

    target_link_libraries(d4 PUBLIC
            flowCutter
            glucose
            ZLIB::ZLIB   # Au lieu de 'z'
            ${GMP_LIBRARY}
    )
else ()
    target_link_libraries(d4 PUBLIC
            z
            gmpxx
            gmp
            glucose
            flowCutter
            ${Boost_LIBRARIES}
    )
endif ()


if (${BUILD_MODE} EQUAL 1)
    message(STATUS "Build D4 : Mode DEBUG (1)")
    if (MSVC)
        target_compile_options(d4 PUBLIC /Zi /Od /U NDEBUG) # /Zi = -g, /Od = no optimization
    else ()
        target_compile_options(d4 PUBLIC -g -O2 -UNDEBUG)
    endif ()
    set(SUFFIX_DIR "-debug")

elseif (${BUILD_MODE} EQUAL 2)
    message(STATUS "Build D4 : Mode STATIC (2)")
    if (MSVC)
        target_compile_options(d4 PUBLIC $<$<NOT:$<CONFIG:Debug>>:/O2> /MT) # MT = static
    else ()
        target_compile_options(d4 PUBLIC -O2)
        target_link_options(d4 PUBLIC -static)
    endif ()
    target_compile_options(d4 PUBLIC -O2)
    target_compile_definitions(d4 PUBLIC NDEBUG)

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
    if (MSVC)
        target_compile_options(d4 PUBLIC $<$<NOT:$<CONFIG:Debug>>:/O2>)
    else ()
        target_compile_options(d4 PUBLIC -O3)
    endif ()
    target_compile_definitions(d4 PUBLIC NDEBUG)
    set(SUFFIX_DIR "-standard")
endif ()

install(TARGETS d4 DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/${CMAKE_PROJECT_NAME}${SUFFIX_DIR})


file(GLOB cmake_generated ${CMAKE_SOURCE_DIR}/src/*~)

foreach (file ${cmake_generated})

    if (EXISTS ${file})
        file(REMOVE_RECURSE ${file})
    endif ()

endforeach (file)