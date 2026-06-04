file(GLOB_RECURSE FLOWCUTTER_SOURCES "src/*.cpp")
add_library(flowCutter OBJECT ${FLOWCUTTER_SOURCES})

if(MSVC)
    target_compile_options(flowCutter PRIVATE /W3 /wd4018)
else()
    target_compile_options(flowCutter PRIVATE -Wall -Wno-sign-compare)
endif()

# Mirror NDEBUG from the parent build so asserts are disabled in release builds.
if (NOT (CMAKE_BUILD_TYPE MATCHES "Debug" OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo" OR BUILD_MODE EQUAL 1))
    target_compile_definitions(flowCutter PRIVATE NDEBUG)
endif()

# Apply optimization flags matching the parent build mode.
if (NOT MSVC)
    if (BUILD_MODE EQUAL 1 OR CMAKE_BUILD_TYPE MATCHES "Debug")
        target_compile_options(flowCutter PRIVATE -O1 -g)
        message(STATUS "FlowCutter: DEBUG (-O1 -g, asserts ON)")
    elseif (BUILD_MODE EQUAL 3 OR CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
        target_compile_options(flowCutter PRIVATE -O3 -g)
        message(STATUS "FlowCutter: PROFILING (-O3 -g, asserts OFF)")
    else()
        target_compile_options(flowCutter PRIVATE -O3 -march=native)
        message(STATUS "FlowCutter: RELEASE (-O3 -march=native, asserts OFF)")
    endif()
endif()

target_include_directories(flowCutter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)