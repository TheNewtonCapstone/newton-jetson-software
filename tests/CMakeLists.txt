find_package(Catch2 3 REQUIRED)

add_executable(tests
  utils/test_logger.cpp
  utils/test_message.cpp
)

target_link_libraries(tests
    PRIVATE
        utils
        communication
        Catch2::Catch2WithMain
)

target_include_directories(tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/utils/include
)

include(Catch)
catch_discover_tests(tests)
message(STATUS "Tests: ${CMAKE_CURRENT_SOURCE_DIR}")