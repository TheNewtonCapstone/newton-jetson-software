add_library(utils
  src/logger.cpp
)

target_include_directories(utils
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()