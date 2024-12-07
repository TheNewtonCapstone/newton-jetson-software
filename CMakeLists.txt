cmake_minimum_required(VERSION 3.10)

# Define the project name
project(Controller)

# Set the C++ standard (optional, depending on your requirement)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# target_compile_features(model PUBLIC c_std_99 cxx_std_17)
# target_compile_features(pid PUBLIC c_std_99 cxx_std_17)


set (SOURCES
      main.cpp
)

set (TESTS
    tests/logger.cpp
)
# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})
add_executable(tests ${TESTS})


include_directories(${CMAKE_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(tests PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)