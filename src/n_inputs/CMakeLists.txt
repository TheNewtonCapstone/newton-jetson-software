cmake_minimum_required(VERSION 3.8)
project(n_inputs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# create an executable
add_executable(keyboard_node 
  src/keyboard_entry.cpp
  src/nodes/keyboard_node.cpp
)

# link ROS dependencies
ament_target_dependencies(
  keyboard_node 
  rclcpp 
  std_msgs 
  geometry_msgs
)

# get the utils directory
set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
cmake_path(GET PROJECT_DIR PARENT_PATH GRANDPARENT_DIR)
cmake_path(GET GRANDPARENT_DIR PARENT_PATH PARENT_DIR)
set(ROOT_PATH ${PARENT_DIR})
set(UTILS_DIR ${PARENT_DIR}/utils)

# include directories
target_include_directories(
  keyboard_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UTILS_DIR}/include
)

# link against libraries
target_link_libraries(
  keyboard_node
  ${UTILS_DIR}/build/libutils.so
)

# install lib
install(TARGETS keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

# install share
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
