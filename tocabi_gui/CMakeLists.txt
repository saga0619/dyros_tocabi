cmake_minimum_required(VERSION 2.8.3)
project(tocabi_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  std_msgs
  geometry_msgs
  mujoco_ros_msgs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()


catkin_python_setup()

roslint_cpp()




catkin_package(
# INCLUDE_DIRS ${tocabi_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp sensor_msgs std_msgs geometry_msgs mujoco_ros_msgs 
#  DEPENDS system_lib
)



set(tocabi_gui_SRCS
  src/tocabi_gui.cpp
)

set(tocabi_gui_HDRS
  include/tocabi_gui/tocabi_gui.h
)

set(tocabi_gui_UIS
  src/tocabi_gui.ui
)




if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(tocabi_gui_MOCS ${tocabi_gui_HDRS})
  qt5_wrap_ui(tocabi_gui_UIS_H ${tocabi_gui_UIS})
else()
  qt4_wrap_cpp(tocabi_gui_MOCS ${tocabi_gui_HDRS})
  qt4_wrap_ui(tocabi_gui_UIS_H ${tocabi_gui_UIS})
endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
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
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()
