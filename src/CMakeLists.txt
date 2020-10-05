cmake_minimum_required(VERSION 3.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# source/include files
file(GLOB SOURCES ./**.cpp)
file(GLOB INCLUDES ./**.hpp)

set(SRCS ${SOURCES})
set(HEADERS ${INCLUDES})

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# flags
if(${DEBUG_MODE})
  # create an executable
  add_executable(d4_debug ${SRCS} ${HEADERS})

  # PROFILE
  message("Debug mode activated")
  set(CMAKE_BUILD_TYPE Debug)
  target_compile_options(d4_debug PUBLIC -std=c++11 -Wall -lz -pg -lgmpxx -lgmp -O2)
  target_link_libraries (d4_debug -lz -pg -lgmpxx -lgmp -O2 ${Boost_LIBRARIES})

  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4_debug
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4_debug)
else()
  # create an executable
  add_executable(d4 ${SRCS} ${HEADERS})

  message("Classical mode activated")
  target_compile_options(d4 PUBLIC -std=c++11 -Wall -lz -O3 -lgmpxx -lgmp)
  target_link_libraries (d4 -lz -O3 -lgmpxx -lgmp ${Boost_LIBRARIES})

  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4)
endif()
