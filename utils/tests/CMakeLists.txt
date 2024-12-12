find_package(Catch2 3 REQUIRED)

add_executable(test_utils
    test_logger.cpp
)

target_link_libraries(test_utils
    PRIVATE
        utils
        Catch2::Catch2WithMain
)

target_include_directories(test_utils
    PRIVATE
        ${CMAKE_SOURCE_DIR}/utils/include
)

include(Catch)
catch_discover_tests(test_utils)