file(GLOB GLUCOSE_SOURCES
        core/Solver.cc
        utils/System.cc
        utils/Options.cc
        "mtl/*.cc"
)

add_library(glucose OBJECT ${GLUCOSE_SOURCES})

target_link_libraries(glucose PRIVATE ZLIB::ZLIB)

target_compile_definitions(glucose PRIVATE
        __STDC_LIMIT_MACROS
        __STDC_FORMAT_MACROS
)

# Mirror NDEBUG from the parent build so asserts are disabled in release builds.
if (NOT (CMAKE_BUILD_TYPE MATCHES "Debug" OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo" OR BUILD_MODE EQUAL 1))
    target_compile_definitions(glucose PRIVATE NDEBUG)
endif()

# Apply optimization flags matching the parent build mode.
if (NOT MSVC)
    if (BUILD_MODE EQUAL 1 OR CMAKE_BUILD_TYPE MATCHES "Debug")
        target_compile_options(glucose PRIVATE -O1 -g)
        message(STATUS "Glucose: DEBUG (-O1 -g, asserts ON)")
    elseif (BUILD_MODE EQUAL 3 OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        target_compile_options(glucose PRIVATE -O3 -g)
        message(STATUS "Glucose: PROFILING (-O3 -g, asserts OFF)")
    elseif (BUILD_MODE EQUAL 4)
        target_compile_options(glucose PRIVATE -O3 -march=native -flto=auto -ffat-lto-objects)
        message(STATUS "Glucose: COMPETITION (-O3 -march=native -flto, asserts OFF)")
    else()
        target_compile_options(glucose PRIVATE -O3 -march=native)
        message(STATUS "Glucose: RELEASE (-O3 -march=native, asserts OFF)")
    endif()
endif()

target_include_directories(glucose PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})