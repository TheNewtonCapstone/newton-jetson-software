# motor_driver CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(n_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ONNXRUNTIME_INCLUDE_DIR /usr/local/include/onnxruntime)
set(ONNXRUNTIME_BIN_DIR /usr/local/lib)

# Onnxruntime
include_directories(include "${ONNXRUNTIME_INCLUDE_DIR}/")

# get the utils directory
set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(GET PROJECT_DIR PARENT_PATH GRANDPARENT_DIR)
cmake_path(GET GRANDPARENT_DIR PARENT_PATH PARENT_DIR)
set(ROOT_PATH ${PARENT_DIR})
set(UTILS_DIR ${PARENT_DIR}/utils)

# Add include directories
include_directories(include)
include_directories(${UTILS_DIR}/include)
include_directories(${UTILS_DIR}/build)

# Define executable target
add_executable(controller_node
  src/main.cpp
  src/gaits/base_gait.cpp
  src/gaits/harmonic_gait.cpp
  src/gaits/machine_gait.cpp
  src/gaits/sliding_gait.cpp
  src/handlers/onnx_handler.cpp
)

message("Grandparent Dir: ${PARENT_DIR}")
message("Parent Dir: ${GRANDPARENT_DIR}")
message("Current Dir: ${PROJECT_DIR}")
message("Utils Include Dir: ${UTILS_DIR}")

# Add ROS dependencies
ament_target_dependencies(controller_node
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
)

target_include_directories(controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${UTILS_DIR}/include  
)

target_link_libraries(controller_node
  ${UTILS_DIR}/build/libutils.so
  onnxruntime
)

# Set compile features
target_compile_features(controller_node PRIVATE cxx_std_20)

install(
  TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
