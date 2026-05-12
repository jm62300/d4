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

target_include_directories(glucose PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})