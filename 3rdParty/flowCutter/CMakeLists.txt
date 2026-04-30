file(GLOB_RECURSE FLOWCUTTER_SOURCES "src/*.cpp")
add_library(flowCutter OBJECT ${FLOWCUTTER_SOURCES})

if(MSVC)
    target_compile_options(flowCutter PRIVATE /W3 /wd4018)
else()
    target_compile_options(flowCutter PRIVATE -Wall -Wno-sign-compare)
endif()

target_include_directories(flowCutter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)