cmake_minimum_required(VERSION 2.8.3)
project(tocabi_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_LIST_SIZE=40)
ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_VECTOR_SIZE=40)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf
  eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

add_message_files(
  FILES
  GainCommand.msg
  MotorInfo.msg
  TaskCommand.msg
  TaskCommandQue.msg
  TaskGainCommand.msg
  model.msg
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

set(CC_PATH ${CMAKE_SOURCE_DIR}/dyros_cc/src)

include_directories(
  include
  ${CC_PATH}
  /usr/local/include/soem
  /usr/share/c++-mscl/source
  /usr/share/c++-mscl/Boost/include
  ${catkin_INCLUDE_DIRS}
)


link_directories(
  /usr/share/c++-mscl
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
  src/walking_controller.cpp
  src/walking_pattern.cpp
  src/wholebody_controller.cpp
  ${CC_PATH}/custom_controller.cpp
  src/mx5_imu.cpp
)

add_executable(tocabi_controller src/base.cpp)

target_link_libraries(tocabi_controller_lib
  ${catkin_LIBRARIES}
  -lmscl
  -lrbdl
  -lrbdl_urdfreader
  -lurdfdom_model
  -lboost_system 
  -lrt
  -lqpOASES
  -lsoem
  -lcanlib
  -lgsl
  -lgslcblas
  ${CMAKE_CURRENT_SOURCE_DIR}/include/sensoray826/lib/lib826_64.so
)

target_link_libraries(tocabi_controller
  tocabi_controller_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(tocabi_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(tocabi_controller &{catkin_EXPORTED_TARGETS})