cmake_minimum_required(VERSION 3.10)

# Define the project name
project(newton)

# Set the C++ standard (optional, depending on your requirement)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# target_compile_features(model PUBLIC c_std_99 cxx_std_17)
# target_compile_features(pid PUBLIC c_std_99 cxx_std_17)

option(ENABLE_TESTING "Enable Test Builds" ON)

add_subdirectory(utils)
add_subdirectory(communication)


add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE
    utils
    communication
)



target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
)

if(ENABLE_TESTING)
    add_subdirectory(tests)
endif()