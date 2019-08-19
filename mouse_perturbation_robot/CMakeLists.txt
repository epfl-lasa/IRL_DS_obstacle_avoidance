cmake_minimum_required(VERSION 2.8.3)

project(mouse_perturbation_robot)

# Add support for c++11
add_definitions(-std=c++11)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(
   DIRECTORY msg
   FILES
   MouseMsg.msg
   MouseMsgPassIRL.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/mouseInterface_params.cfg cfg/obstacleAvoidance_params.cfg)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure geometry_msgs
  # DEPENDS Eigen3
 )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(motionGenerator src/motionGenerator_node.cpp src/MotionGenerator.cpp src/DSObstacleAvoidance.cpp)
target_link_libraries(motionGenerator ${catkin_LIBRARIES})
add_dependencies(motionGenerator ${PROJECT_NAME}_gencpp)

add_executable(mouseInterface src/mouseInterface_node.cpp src/MouseInterface.cpp)
target_link_libraries(mouseInterface ${catkin_LIBRARIES})
add_dependencies(mouseInterface ${PROJECT_NAME}_gencpp)