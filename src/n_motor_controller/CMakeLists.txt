# motor_driver CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(n_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(${UTILS_DIR}/build)


# Try to find odrive_can package
find_package(odrive_can QUIET)
if(NOT odrive_can_FOUND)
  message(WARNING "odrive_can package not found. Make sure it's built and sourced properly.")
  # You might want to specify the path manually if needed:
  # set(odrive_can_DIR "/path/to/odrive_can/install/share/odrive_can/cmake")
  # find_package(odrive_can UIRED)
endif()

# get the utils directory
set(MY_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(GET MY_PROJECT_DIR PARENT_PATH level1)
cmake_path(GET level1 PARENT_PATH level2)
set(ROOT_PATH ${level2})
set(UTILS_DIR ${level2}/utils)

# Add include directories
include_directories(include)
include_directories(($UTILS_DIR)/include)


# Define executable target
add_executable(motor_driver_node
src/motor.cpp
src/main.cpp
)



message(WARNING "grandparent dir: ${level2}")
message(WARNING "parent dir: ${level1}")
message(WARNING "current dir: ${MY_PROJECT_DIR}")
message(WARNING "utils include dir: ${UTILS_DIR}")

# Add dependencies
ament_target_dependencies(motor_driver_node
  rclcpp
  rclcpp_components
  odrive_can
  std_srvs
)

target_include_directories(motor_driver_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${UTILS_DIR}/include  

)

target_link_libraries(motor_driver_node
  # ${ROOT_PATH}/build/libutils.so 
  ${UTILS_DIR}/build/libutils.so
)

# Set compile features
target_compile_features(motor_driver_node PRIVATE cxx_std_20)

install(
  TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
