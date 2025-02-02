# root 
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(newton)

# Set the C++ standard (optional, depending on your requirement)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# target_compile_features(model PUBLIC c_std_99 cxx_std_17)
# target_compile_features(pid PUBLIC c_std_99 cxx_std_17)

# option(ENABLE_TESTING "Enable Test Builds" ON)


find_package(ament_cmake REQUIRED)
add_subdirectory(src/utils)
add_subdirectory(src/motor_driver)
add_subdirectory(src/imu)


add_executable(${PROJECT_NAME} main.cpp)

install(
 DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()