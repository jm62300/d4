file(GLOB GLUCOSE_BIPE_SOURCES
        3rdParty/glucose-3.0/core/Solver.cc
        3rdParty/glucose-3.0/utils/System.cc
        3rdParty/glucose-3.0/utils/Options.cc
        "3rdParty/glucose-3.0/mtl/*.cc"
)
add_library(glucose_bipe STATIC ${GLUCOSE_BIPE_SOURCES})
target_link_libraries(glucose_bipe PRIVATE ZLIB::ZLIB)
target_compile_definitions(glucose_bipe PRIVATE __STDC_LIMIT_MACROS __STDC_FORMAT_MACROS)
target_include_directories(glucose_bipe PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0)

file(GLOB_RECURSE BIPE_SOURCES "src/*.cpp")
add_library(bipe STATIC ${BIPE_SOURCES})
target_compile_features(bipe PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(bipe PRIVATE
        /DNOMAIN
        /W3        # -Wall
        /wd4018    # -Wno-sign-compare
        /wd4706    # -Wno-parentheses
        /wd4477    # -Wno-format
        /wd4455    # -Wno-literal-suffix
    )
else()
    target_compile_options(bipe PRIVATE
        -DNOMAIN
        -Wall
        -Wno-class-memaccess
        -Wno-format
        -Wno-misleading-indentation
        -Wno-literal-suffix
        -Wno-sign-compare
        -Wno-parentheses
    )
endif()

target_include_directories(glucose_bipe BEFORE PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0
)

target_include_directories(bipe BEFORE PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glucose-3.0
)

target_link_libraries(bipe PUBLIC ZLIB::ZLIB glucose_bipe)