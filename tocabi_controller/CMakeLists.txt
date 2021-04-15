cmake_minimum_required(VERSION 2.8.3)
project(tocabi_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O1)

ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_LIST_SIZE=40)
ADD_DEFINITIONS(-DBOOST_MPL_LIMIT_VECTOR_SIZE=40)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- kernel version : " ${UNAME_RES})

string(FIND ${UNAME_RES} "xenomai" wsloc)

if(${wsloc} EQUAL -1 )
  message(" simulation only compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_SIMULATION)
  set(COMPILE_SIM TRUE)
else()
  message(" xenomai detected realrobot compilt mode ")
  ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
  set(COMPILE_REAL TRUE)
endif() 

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
  mujoco_ros
  eigen_conversions
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(osqp REQUIRED)
add_message_files(
  FILES
  GainCommand.msg
  MotorInfo.msg
  TaskCommand.msg
  TaskCommandQue.msg
  TaskGainCommand.msg
  positionCommand.msg
  torqueCommand.msg
  model.msg
  VelocityCommand.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)
catkin_package()

set(CC_PATH ${CMAKE_SOURCE_DIR}/dyros_cc/src)

if(${COMPILE_REAL})
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
      src/osqp_rapper.cpp
    )
    add_executable(tocabi_controller src/base.cpp)
    target_link_libraries(tocabi_controller_lib
      ${catkin_LIBRARIES}
      -losqp
      -lmscl
      -lrbdl
      -lrbdl_urdfreader
      -lboost_system 
      -lrt
      -lqpOASES
      -lsoem
      -lcanlib
      -lgsl
      -lgslcblas
      ${CMAKE_CURRENT_SOURCE_DIR}/include/sensoray826/lib/lib826_64.so
    )
endif()

if(${COMPILE_SIM})
    include_directories(
      include
      ${CC_PATH}
      ${catkin_INCLUDE_DIRS}
    )
    add_library(tocabi_controller_lib
      src/link.cpp
      src/terminal.cpp
      src/dynamics_manager.cpp
      src/qp.cpp
      src/state_manager.cpp
      src/mujoco_interface.cpp
      src/tocabi_controller.cpp
      src/walking_controller.cpp
      src/walking_pattern.cpp
      src/wholebody_controller.cpp
      ${CC_PATH}/custom_controller.cpp
      src/osqp_rapper.cpp
    )
    add_executable(tocabi_controller src/base.cpp)
    target_link_libraries(tocabi_controller_lib
      ${catkin_LIBRARIES}
      -losqp
      -lrbdl
      -lrbdl_urdfreader
      -lboost_system 
      -lrt
      -lqpOASES
      -lgsl
      -lgslcblas
    )
endif()





target_link_libraries(tocabi_controller
  tocabi_controller_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(tocabi_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(tocabi_controller &{catkin_EXPORTED_TARGETS})
