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


add_subdirectory(src/utils)


add_executable(${PROJECT_NAME} main.cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(odrive_can)
find_package(motor_driver REQUIRED)
find_package(tm_imu REQUIRED)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE
    utils
    ${rclcpp_components_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_srvs_LIBRARIES}
    ${odrive_can_LIBRARIES}
    ${motor_driver_LIBRARIES}
    ${tm_imu_LIBRARIES}
)




target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
    ${rclcpp_components_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
    ${odrive_can_INCLUDE_DIRS}
    ${motor_driver_INCLUDE_DIRS}
    ${tm_imu_INCLUDE_DIRS}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()