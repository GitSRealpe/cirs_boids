cmake_minimum_required(VERSION 3.0.2)
project(cirs_boids)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_STANDARD 11)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   set(CMAKE_CXX_EXTENSIONS OFF)
#   # this next line shouldn't be necessary, but doesn't always get added by cmake (e.g., for clang++-5)
#   add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}")
  
include(./macros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Stonefish)

list(APPEND CATKIN_SOURCES
    roscpp
    rospy
    tf
    tf2
    tf2_ros
    visualization_msgs
    std_msgs
    geometry_msgs
    rosbag
    rviz_visual_tools
    stonefish_ros
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation actionlib genmsg)

# add_service_files(
#   FILES
#   Behavior.srv
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# when having setup.py
# catkin_python_setup()

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

###########s
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

include_directories(SYSTEM ${INCLUDES})

list(APPEND LIBRARIES
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  
)

add_library(${PROJECT_NAME}
  src/Flock.cpp
)

# target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3  ${catkin_LIBRARIES})

# add_dependencies(cirs_boids ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cirs_boids ${catkin_LIBRARIES} ${LIBRARIES})

add_executable(viz_boids scripts/viz_boids.cpp)
target_link_libraries(viz_boids cirs_boids)

add_executable(stonefish_boids scripts/stonefish_boids.cpp)
target_link_libraries(stonefish_boids cirs_boids ${catkin_LIBRARIES} ${LIBRARIES} Stonefish::Stonefish)

# add_script(test scripts)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination

# catkin_install_python(PROGRAMS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
