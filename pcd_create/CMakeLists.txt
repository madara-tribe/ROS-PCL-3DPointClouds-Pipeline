cmake_minimum_required(VERSION 3.0.2)
project(pcd_create)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  std_msgs
)

find_package(PCL 1.8 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd_create
#  CATKIN_DEPENDS cv_bridge image_transport message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(vccs
  src/vccs.cpp
)
target_link_libraries(vccs
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(save_pcd
  src/save_pcd.cpp
)
target_link_libraries(save_pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

