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

target_include_directories(utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

message(STATUS "utils include each directory: ${CMAKE_CURRENT_SOURCE_DIR}/include")
message(STATUS "utils source dir: ${CMAKE_CURRENT_SOURCE_DIR}/src")

