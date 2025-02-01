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

# add_library(motor_driver_node)
# target_sources(motor_driver_node
#   PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/motor.cpp
#     PUBLIC 
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/motor_driver/motor.h
# )

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


# target_include_directories(motor_driver_node
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# Define executable target
add_executable(motor_driver_node
src/motor.cpp
src/main.cpp
)

# add the utils library
# set(utils_DIR "${CMAKE_SOURCE_DIR}/../../utils")
# target_include_directories(motor_driver_node ${CMAKE_SOURCE_DIR}/../../utils/include)
# message(WARNING "utils include dir: ${utils_DIR}")


# Link libraries
set(MY_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(GET MY_PROJECT_DIR PARENT_PATH result)
message(WARNING "utils include dir: ${result}")
# include_directories(${result}/utils/include)

# Add dependencies
ament_target_dependencies(motor_driver_node
  rclcpp
  rclcpp_components
  odrive_can
  std_srvs
)

# target_link_libraries(motor_driver_node
#   utils
# )

# Set compile features
target_compile_features(motor_driver_node PRIVATE cxx_std_20)

install(
  TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# If you have a launch directory, uncomment this
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
