cmake_minimum_required(VERSION 2.8.3)
project(miro_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  DIRECTORY
  srv
  FILES
  GestureProcessing.srv
  MonteCarlo.srv
  SpatialReasoner.srv
  PertinenceMapping.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

add_executable(gesture_processing_server src/gesture_processing_server.cpp)
target_link_libraries(gesture_processing_server ${catkin_LIBRARIES})
add_dependencies(gesture_processing_server miro_teleop_gencpp)

add_executable(gesture_processing_client src/gesture_processing_client.cpp)
target_link_libraries(gesture_processing_client ${catkin_LIBRARIES})
add_dependencies(gesture_processing_client miro_teleop_gencpp)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
