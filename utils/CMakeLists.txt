add_library(utils)

file(GLOB_RECURSE HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)

file(GLOB_RECURSE SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# header files here
target_sources(utils
  PUBLIC
    ${HEADER_FILES}
  PRIVATE 
    ${SOURCE_FILES}
)

# If there are no source files, create a dummy file to define the language
# this is a workaround for CMake not allowing targets with no source files because it doesnt know the language
# TODO : update documentation to refect the new cmake list file
if(NOT SOURCE_FILES)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/src/dummy.cpp "// Dummy file to define language\n")
    set(SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/dummy.cpp)
endif()

set_target_properties(utils PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

message(STATUS "utils include each directory: ${CMAKE_CURRENT_SOURCE_DIR}/include")
message(STATUS "utils source dir: ${CMAKE_CURRENT_SOURCE_DIR}/src")

