# motor_driver CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

# Try to find odrive_can package
find_package(odrive_can QUIET)
if(NOT odrive_can_FOUND)
  message(WARNING "odrive_can package not found. Make sure it's built and sourced properly.")
  # You might want to specify the path manually if needed:
  # set(odrive_can_DIR "/path/to/odrive_can/install/share/odrive_can/cmake")
  # find_package(odrive_can UIRED)
endif()

# Add include directories
include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/motor.cpp
)

# link against utils library
target_link_libraries(${PROJECT_NAME} 
    PRIVATE
    utils
)
# Define executable target
add_executable(motor_driver_node
    src/motor.cpp
    src/main.cpp
)

# Add dependencies
ament_target_dependencies(motor_driver_node
  rclcpp
  rclcpp_components
  odrive_can
  std_srvs
)

# Set compile features
target_compile_features(motor_driver_node PRIVATE cxx_std_20)

# Installation
install(
  TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# If you have a launch directory, uncomment this
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
