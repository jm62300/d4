cmake_minimum_required(VERSION 3.1)

# set the project name and version
project(D4 VERSION 0.1)

add_custom_target(clean-cmake-files
   COMMAND ${CMAKE_COMMAND} -P clean-all.cmake
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_BUILD_TYPE Release)
add_subdirectory(src)

file(GLOB cmake_generated ${CMAKE_SOURCE_DIR}/src/*~)



foreach(file ${cmake_generated})

  if (EXISTS ${file})
     file(REMOVE_RECURSE ${file})
  endif()

endforeach(file)
