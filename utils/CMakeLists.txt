cmake_minimum_required(VERSION 3.8)
project(utils)

# Make sure to build as a shared library
add_library(utils SHARED)

file(GLOB_RECURSE HEADER_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)

file(GLOB_RECURSE SOURCE_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

target_sources(utils
  PUBLIC
    ${HEADER_FILES}
  PRIVATE 
    ${SOURCE_FILES}
)

# Make includes public for linking libraries
target_include_directories(utils
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Export include directories
set(utils_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "")
