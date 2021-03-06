cmake_minimum_required(VERSION 2.8.3)
project(hextree)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  image_transport
)

## System dependencies are found with CMake's conventions#
find_package(Eigen3 REQUIRED)
find_package(OpenCV)

#include_directories(${CMAKE_CURRENT_BINARY_DIR})

#add service
add_service_files(
  DIRECTORY srv
  FILES
  pidgain.srv
  plannertalk.srv
  obstacle.srv
  sensor.srv
  measurement.srv

)

## Generate added messages
generate_messages(DEPENDENCIES std_msgs)


# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})



catkin_package(

 CATKIN_DEPENDS image_transport sensor_msgs cv_bridge  message_runtime
std_msgs

)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${QT_INCLUDES}
${CMAKE_CURRENT_BINARY_DIR}
)

# project libraries
set (PROJECT_LIB
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )



# controller
set(CONTROLLER_HEADERS
  controller/controller/controller.h
  controller/controller/trajectory.h
  controller/controller/seeker.h

  controller/stateEstimator/stateestimation.h
  controller/stateEstimator/scale.h
  
#  controller/mav.h
  controller/header.h
  controller/comm_function.h
  hexTree/datalogger.h

  controller/visualizer/display.h



)
set(CONTROLLER_SOURCES
  controller/controller/controller.cpp
  controller/controller/trajectory.cpp
  controller/controller/seeker.cpp
  
  controller/stateEstimator/stateestimation.cpp
  controller/stateEstimator/scale.cpp
  

  controller/main.cpp
#  controller/mav.cpp
  hexTree/datalogger.cpp

  controller/visualizer/display.cpp


)

# build!
 add_executable(motion1 ${CONTROLLER_HEADERS} ${CONTROLLER_SOURCES})
 target_link_libraries(motion1 ${PROJECT_LIB})
add_dependencies(motion1 ${PROJECT_NAME}_gencfg)


# # ---------------------------- HOTSPOT SEEKING --------------------------------------------------


#path planner
set(HEXSEARCH_HEADERS
 hexTree/infotree.h
 hexTree/display.h
 hexTree/dpsolver.h
 hexTree/hotspot.h
 hexTree/coverage.h
 hexTree/datalogger.h

)
set(HEXSEARCH_SOURCES
 hexTree/infotree.cpp
 hexTree/display.cpp
 hexTree/dpsolver.cpp
 hexTree/main.cpp
 hexTree/hotspot.cpp
 hexTree/coverage.cpp
 hexTree/datalogger.cpp
)
# build!
add_executable(seeking1 ${HEXSEARCH_HEADERS} ${HEXSEARCH_SOURCES})
target_link_libraries(seeking1 ${PROJECT_LIB})


# # ---------------------------- GUI --------------------------------------------------

set(GUI_SOURCE_FILES         
  gui/main.cpp
  gui/mainwindow.cpp
  gui/ros_thread.cpp
  gui/ros_launch.cpp

  controller/visualizer/qcustomplot.cpp
  controller/visualizer/robotviz.cpp
)

set(GUI_HEADER_FILES
  gui/mainwindow.h      
  gui/ros_thread.h
  gui/ros_launch.h
  gui/header.h

  controller/visualizer/qcustomplot.h
  controller/visualizer/robotviz.h
)

# *.ui
set(GUI_UI_FILES        
  gui/mainwindow.ui
  controller/visualizer/robotviz.ui
)

# *.qrc
set(GUI_RESOURCE_FILES
    gui/parameter.qrc
)


QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})

# # build!
add_executable(gui1 ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(gui1 ${PROJECT_LIB}
)
add_dependencies(gui1 ${PROJECT_NAME}_gencfg)


