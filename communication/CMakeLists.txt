add_library(communication)

file(GLOB_RECURSE HEADER_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)

file(GLOB_RECURSE SOURCE_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

target_sources(communication
    PUBLIC
        ${HEADER_FILES}
    PRIVATE
        ${SOURCE_FILES}
)

target_include_directories(communication
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(communication
    PUBLIC
        utils
)

message(STATUS "Headers: ${HEADER_FILES}")
message(STATUS "Sources: ${SOURCE_FILES}")