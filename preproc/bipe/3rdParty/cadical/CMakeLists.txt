file(GLOB CADICAL_SOURCES 
    src/*.cpp
    src/*.c
)

# Exclude applications
list(REMOVE_ITEM CADICAL_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cadical.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mobical.cpp"
)

add_library(cadical OBJECT ${CADICAL_SOURCES})

target_include_directories(cadical PUBLIC src)

target_compile_definitions(cadical PRIVATE 
    NDEBUG 
    NBUILD
)

if(WIN32)
    target_compile_definitions(cadical PRIVATE _WIN32)
endif()

if(MSVC)
    target_compile_options(cadical PRIVATE /W3)
    # CaDiCaL uses some POSIX names like 'access', 'popen', etc.
    target_compile_definitions(cadical PRIVATE _CRT_NONSTDC_NO_DEPRECATE _CRT_SECURE_NO_WARNINGS)
endif()
