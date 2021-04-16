cmake_minimum_required(VERSION 3.1)

# set the project name and version
project(D4 VERSION 0.1)

add_custom_target(clean-cmake-files
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_BUILD_TYPE Release)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# source/include files
file(GLOB_RECURSE SOURCES ./src/**.cpp)
file(GLOB_RECURSE INCLUDES ./src/**.hpp)

set(SRCS ${SOURCES})
set(HEADERS ${INCLUDES})

set(Boost_USE_STATIC_LIBS   ON)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

if (NOT DEFINED BUILD_MODE)
   set(BUILD_MODE 1)
endif()


# flags
if(${BUILD_MODE} EQUAL 1)
  # create an executable
  add_executable(d4_debug ${SRCS} ${HEADERS})

  # PROFILE
  message("Debug mode activated")
  set(CMAKE_BUILD_TYPE Debug)
  target_compile_options(d4_debug PUBLIC -std=c++11 -Wall -pg -O2 -Wno-class-memaccess -D DEBUG)
  target_link_libraries (d4_debug -lz -pg -lgmpxx -lgmp -O2 ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a)

  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4_debug
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4_debug)
elseif(${BUILD_MODE} EQUAL 2)
  message("Static mode activated")

  add_executable(d4_static ${SRCS} ${HEADERS})

  target_compile_options(d4_static PUBLIC -std=c++11 -Wall -O3 -Wno-class-memaccess -D NDEBUG -static)
   target_link_libraries (d4_static -lz -O3 -lgmpxx -lgmp ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a -D NDEBUG -static)
  
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4_static
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4_static)
else()  
  # create an executable
  add_executable(d4 ${SRCS} ${HEADERS})

  message("Classical mode activated")
  target_compile_options(d4 PUBLIC -std=c++11 -Wall -O3 -Wno-class-memaccess -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
  target_link_libraries (d4 -lz -O3 -lgmpxx -lgmp ${Boost_LIBRARIES} ${CMAKE_SOURCE_DIR}/3rdParty/patoh/libpatoh.a)

  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/d4
    DESTINATION bin
    RENAME ${CMAKE_PROJECT_NAME}-d4)
endif()


file(GLOB cmake_generated ${CMAKE_SOURCE_DIR}/src/*~)

foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)



#
# Testing
#

# enable_testing()

# include(cmake_includes/AddGoogleTest.cmake)
# add_executable(HashString_test tests/HashString_test.cpp)
# add_gtest(HashString_test)
