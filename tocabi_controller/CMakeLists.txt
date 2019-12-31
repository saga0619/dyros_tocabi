cmake_minimum_required(VERSION 2.8.3)
project(tocabi_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  eigen_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(
  FILES
  GainCommand.msg
  MotorInfo.msg
  TaskCommand.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tocabi_controller
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp roslib sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  /usr/local/include/soem
  ${catkin_INCLUDE_DIRS}
)

add_library(tocabi_controller_lib
  src/link.cpp
  src/terminal.cpp
  src/dynamics_manager.cpp
  src/qp.cpp
  src/realrobot_interface.cpp
  src/state_manager.cpp
  src/mujoco_interface.cpp
  src/tocabi_controller.cpp
  src/wholebody_controller.cpp
)

add_executable(tocabi_controller src/controller.cpp)

target_link_libraries(tocabi_controller_lib
  ${catkin_LIBRARIES}
  -lrbdl
  -lrbdl_urdfreader
  -lrt
  -lqpOASES
  -lncurses
  -lsoem
  ${CMAKE_CURRENT_SOURCE_DIR}/include/sensoray826/lib/lib826_64.so
)

target_link_libraries(tocabi_controller
  tocabi_controller_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(tocabi_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(tocabi_controller &{catkin_EXPORTED_TARGETS})
