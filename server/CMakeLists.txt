cmake_minimum_required(VERSION 3.14)
project(D4GrpcServer VERSION 0.1 LANGUAGES CXX)

# Specify C++20 for compatibility with d4 library
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- 1. Find or Download gRPC/Protobuf ---
find_package(gRPC CONFIG QUIET)
find_package(Protobuf CONFIG QUIET)

if (gRPC_FOUND AND Protobuf_FOUND)
    message(STATUS "Found system gRPC and Protobuf packages.")
    find_program(PROTOC_PATH NAMES protoc REQUIRED)
    find_program(GRPC_CPP_PLUGIN_PATH NAMES grpc_cpp_plugin REQUIRED)
    set(GRPC_LIBRARIES gRPC::grpc++ gRPC::grpc++_reflection protobuf::libprotobuf)
else()
    message(STATUS "gRPC/Protobuf not found on system. Fetching from GitHub (v1.62.0)...")
    include(FetchContent)
    
    # Configure gRPC options to minimize compilation time
    set(gRPC_BUILD_TESTS OFF CACHE BOOL "Disable tests" FORCE)
    set(gRPC_INSTALL OFF CACHE BOOL "Disable install targets" FORCE)
    set(protobuf_BUILD_TESTS OFF CACHE BOOL "Disable protobuf tests" FORCE)
    set(protobuf_INSTALL OFF CACHE BOOL "Disable protobuf installation" FORCE)
    set(gRPC_BUILD_CSHARP_EXT OFF CACHE BOOL "Disable C# extension" FORCE)
    set(gRPC_BUILD_GRPC_CSHARP_PLUGIN OFF CACHE BOOL "Disable C# plugin" FORCE)
    set(gRPC_BUILD_GRPC_NODE_PLUGIN OFF CACHE BOOL "Disable Node plugin" FORCE)
    set(gRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN OFF CACHE BOOL "Disable Objective-C plugin" FORCE)
    set(gRPC_BUILD_GRPC_PHP_PLUGIN OFF CACHE BOOL "Disable PHP plugin" FORCE)
    set(gRPC_BUILD_GRPC_PYTHON_PLUGIN OFF CACHE BOOL "Disable Python plugin" FORCE)
    set(gRPC_BUILD_GRPC_RUBY_PLUGIN OFF CACHE BOOL "Disable Ruby plugin" FORCE)
    
    FetchContent_Declare(
        gRPC
        GIT_REPOSITORY https://github.com/grpc/grpc.git
        GIT_TAG        v1.62.0
        GIT_SUBMODULES_RECURSE ON
    )
    FetchContent_MakeAvailable(gRPC)
    
    set(PROTOC_PATH $<TARGET_FILE:protoc>)
    set(GRPC_CPP_PLUGIN_PATH $<TARGET_FILE:grpc_cpp_plugin>)
    set(GRPC_LIBRARIES grpc++ grpc++_reflection libprotobuf)
endif()

# --- 2. Fetch Dependencies (optree & bipe) ---
include(FetchContent)
if(NOT DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(ENV{GITLAB_TOKEN_LOGICAL} "glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko")
endif()

# optree
set(OPTREE_GIT_URL "https://gitlab.univ-artois.fr/logical/optree.git")
if(DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(OPTREE_GIT_URL "https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/optree.git")
endif()
FetchContent_Declare(
    optree
    GIT_REPOSITORY ${OPTREE_GIT_URL}
    GIT_TAG        main
)
FetchContent_MakeAvailable(optree)

# bipe
set(BIPE_GIT_URL "https://gitlab.univ-artois.fr/logical/bipe.git")
if(DEFINED ENV{GITLAB_TOKEN_LOGICAL})
    set(BIPE_GIT_URL "https://oauth2:$ENV{GITLAB_TOKEN_LOGICAL}@gitlab.univ-artois.fr/logical/bipe.git")
endif()
FetchContent_Declare(
    bipe
    GIT_REPOSITORY ${BIPE_GIT_URL}
    GIT_TAG        main
)
FetchContent_MakeAvailable(bipe)

# --- 3. Find D4 Library and Dependencies ---
get_filename_component(D4_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../" ABSOLUTE)
get_filename_component(D4_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../build" ABSOLUTE)
set(OPTREE_INCLUDE_DIR "${optree_SOURCE_DIR}/include")

find_library(D4_LIB 
    NAMES d4 libd4
    PATHS "${D4_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED
)

# Standard dependencies of D4
find_package(ZLIB REQUIRED)
find_library(GMP_LIB NAMES gmp libgmp REQUIRED)
find_library(GMPXX_LIB NAMES gmpxx libgmpxx REQUIRED)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PATOH_DIR "${D4_ROOT_DIR}/3rdParty/patoh")
    find_library(PATOH_LIBRARY 
        NAMES patoh libpatoh 
        PATHS ${PATOH_DIR} 
        NO_DEFAULT_PATH 
        REQUIRED
    )
endif()

# --- 4. Gather Parser and Semiring Directories ---
get_filename_component(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../c++/parser" ABSOLUTE)
get_filename_component(SEMIRINGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../c++/semirings" ABSOLUTE)

file(GLOB_RECURSE PARSER_SRCS "${PARSER_DIR}/*.cpp")

# --- 5. Code Generation (Protobuf / gRPC) ---
set(PROTO_SRC "${CMAKE_CURRENT_SOURCE_DIR}/proto/d4.proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/gens")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(D4_PB_SRC "${PROTO_GEN_DIR}/d4.pb.cc")
set(D4_PB_HDR "${PROTO_GEN_DIR}/d4.pb.h")
set(D4_GRPC_SRC "${PROTO_GEN_DIR}/d4.grpc.pb.cc")
set(D4_GRPC_HDR "${PROTO_GEN_DIR}/d4.grpc.pb.h")

add_custom_command(
    OUTPUT "${D4_PB_SRC}" "${D4_PB_HDR}" "${D4_GRPC_SRC}" "${D4_GRPC_HDR}"
    COMMAND ${PROTOC_PATH}
    ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
         --cpp_out=${PROTO_GEN_DIR}
         --grpc_out=${PROTO_GEN_DIR}
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_PATH}
         ${PROTO_SRC}
    DEPENDS "${PROTO_SRC}"
    COMMENT "Generating C++ files from d4.proto"
    VERBATIM
)

# --- 6. Build Targets ---
# Common Include Directories
set(COMMON_INCLUDES
    ${D4_ROOT_DIR}
    ${D4_ROOT_DIR}/src/include
    ${PARSER_DIR}
    ${D4_ROOT_DIR}/c++
    ${OPTREE_INCLUDE_DIR}
    ${PROTO_GEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Common Link Libraries
set(COMMON_LIBS
    ${D4_LIB}
    bipe
    optree
    ${GMPXX_LIB}
    ${GMP_LIB}
    ZLIB::ZLIB
    ${GRPC_LIBRARIES}
    nlohmann_json
)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND COMMON_LIBS ${PATOH_LIBRARY})
endif()

# Executable: Server
add_executable(d4_grpc_server
    src/main.cpp
    "${D4_ROOT_DIR}/api/solver/Solver.cpp"
    ${PARSER_SRCS}
    ${D4_PB_SRC}
    ${D4_PB_HDR}
    ${D4_GRPC_SRC}
    ${D4_GRPC_HDR}
)
target_include_directories(d4_grpc_server PRIVATE ${COMMON_INCLUDES})
target_link_libraries(d4_grpc_server PRIVATE ${COMMON_LIBS})

# Executable: Client
add_executable(d4_grpc_client
    src/client.cpp
    ${D4_PB_SRC}
    ${D4_PB_HDR}
    ${D4_GRPC_SRC}
    ${D4_GRPC_HDR}
)
target_include_directories(d4_grpc_client PRIVATE ${COMMON_INCLUDES})
target_link_libraries(d4_grpc_client PRIVATE ${COMMON_LIBS})

# Executable: KCMCP Server
add_executable(d4_kcmcp_server
    src/kcmcp_server.cpp
    "${D4_ROOT_DIR}/api/solver/Solver.cpp"
    ${PARSER_SRCS}
)
target_include_directories(d4_kcmcp_server PRIVATE ${COMMON_INCLUDES})
target_link_libraries(d4_kcmcp_server PRIVATE ${COMMON_LIBS})

