cmake_minimum_required(VERSION 2.8.3)
project(record_data)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_runtime
  #nav_core
  nav_msgs
  sensor_msgs
  pcl_ros
  pluginlib
  roscpp
  rospy
  std_msgs
  dg_msgs
  tf
  tf2
  tf2_msgs
  tf2_ros
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

add_library(read_txt 
            src/read_txt.cpp
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES read_txt 
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_core nav_msgs pcl_ros pluginlib roscpp rospy std_msgs dg_msgs tf 
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(odom_tf_node src/odom_tf_node.cpp)
target_link_libraries(odom_tf_node ${catkin_LIBRARIES})

add_executable(tf_listen_node src/tf_listen_node.cpp)
target_link_libraries(tf_listen_node ${catkin_LIBRARIES})

add_executable(auto_get_goal_node src/auto_get_goal.cpp)
target_link_libraries(auto_get_goal_node ${catkin_LIBRARIES})

#add_executable(read_laser_node src/read_laser_node.cpp)
#target_link_libraries(read_laser_node ${catkin_std_msgs})
#target_link_libraries(scan_test_node ${catkin_LIBRARIES})

add_executable(scan_data_analyze_node src/scan_data_analyze.cpp)
target_link_libraries(scan_data_analyze_node ${catkin_LIBRARIES})

add_executable(read_pose_cov_node src/read_pose_cov.cpp)
target_link_libraries(read_pose_cov_node ${catkin_LIBRARIES})

add_executable(auto_record_cov_belief_node src/auto_record_cov_belief.cpp)
target_link_libraries(auto_record_cov_belief_node ${catkin_LIBRARIES})

add_executable(pub_pos_from_txt_node src/pub_pos_from_txt.cpp)
target_link_libraries(pub_pos_from_txt_node 
                      read_txt
                      ${catkin_LIBRARIES}
)

add_executable(visual_marker_node src/visual_marker.cpp)
target_link_libraries(visual_marker_node ${catkin_LIBRARIES})

#############
## Install ##
#############
 install(PROGRAMS
   scripts/T_test_with_id_simplify.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS 
    read_txt
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )
  
 install(TARGETS odom_tf_node 
                 tf_listen_node 
                 auto_get_goal_node 
                 #read_laser_node 
                 # scan_test_node 
                 scan_data_analyze_node
                 auto_record_cov_belief_node
                 pub_pos_from_txt_node
                 visual_marker_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


