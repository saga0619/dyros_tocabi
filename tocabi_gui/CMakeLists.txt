cmake_minimum_required(VERSION 2.8.3)
project(tocabi_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mujoco_ros_msgs
  roscpp
  rospy
  rqt_gui
  rqt_gui_py
  sensor_msgs
  std_msgs
)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

##include(${QT_USE_FILE})
SET(CMAKE_AUTOUIC ON)  

set(tocabi_gui_SRCS
  src/tocabi_gui.cpp
)

set(tocabi_gui_HDRS
  include/tocabi_gui/tocabi_gui.h
)

set(tocabi_gui_UIS
  resource/tocabi_gui.ui
)

set(tocabi_gui_INCLUDE_DIRECTORIES
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${tocabi_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs mujoco_ros_msgs roscpp rospy rqt_gui rqt_gui_py sensor_msgs std_msgs
#  DEPENDS system_lib
)
catkin_python_setup()

QT5_WRAP_CPP(tocabi_gui_MOCS ${tocabi_gui_HDRS})
QT5_WRAP_UI(tocabi_gui_UIS_H ${tocabi_gui_UIS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${tocabi_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${tocabi_gui_SRCS}
  ${tocabi_gui_MOCS}
  ${tocabi_gui_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/tocabi_gui
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/tocabi_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
