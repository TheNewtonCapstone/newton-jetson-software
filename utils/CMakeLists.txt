add_library(utils)

# header files here 
target_sources(utils
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/logger.h
)

target_sources(utils
  PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger.cpp
)
target_include_directories(utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

