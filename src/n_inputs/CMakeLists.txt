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

# add lib subdirectories
add_subdirectory(lib/gainput)

# create an executable
add_executable(keyboard_node 
  src/keyboard_entry.cpp
  src/nodes/keyboard_node.cpp
)
add_executable(gamepad_node 
  src/gamepad_entry.cpp
  src/nodes/gamepad_node.cpp
)

# link ROS dependencies
ament_target_dependencies(
  keyboard_node 
  rclcpp 
  std_msgs 
  geometry_msgs
)
ament_target_dependencies(
  gamepad_node 
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
target_include_directories(
  gamepad_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UTILS_DIR}/include
  ${PROJECT_DIR}/lib/gainput/include
)

# link against libraries
target_link_libraries(
  keyboard_node
  ${UTILS_DIR}/build/libutils.so
)

target_link_libraries(
  gamepad_node
  ${UTILS_DIR}/build/libutils.so
  gainputstatic
  ${PROJECT_DIR}/lib/gainput/build/lib/libgainput.so
  X11
)

# install lib
install(TARGETS keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

# install share
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
