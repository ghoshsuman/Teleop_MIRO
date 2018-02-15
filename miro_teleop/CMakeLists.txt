cmake_minimum_required(VERSION 2.8.3)
project(miro_teleop)

set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  miro_msgs
  OpenCV
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
  rrtstar_msgs
  miro_msgs
)

add_executable(interpreter src/interpreter.cpp)
target_link_libraries(interpreter ${catkin_LIBRARIES})
add_dependencies(interpreter miro_teleop_gencpp)

add_executable(command_logic src/command_logic.cpp)
target_link_libraries(command_logic ${catkin_LIBRARIES})
add_dependencies(command_logic miro_teleop_gencpp rrtstar_msgs_gencpp)

add_executable(gesture_processing_server src/gesture_processing.cpp)
target_link_libraries(gesture_processing_server ${catkin_LIBRARIES})
add_dependencies(gesture_processing_server miro_teleop_gencpp)

add_executable(monte_carlo_server src/monte_carlo.cpp)
target_link_libraries(monte_carlo_server ${catkin_LIBRARIES})
add_dependencies(monte_carlo_server miro_teleop_gencpp)

add_executable(pertinence_mapping_server src/pertinence_mapping.cpp)
target_link_libraries(pertinence_mapping_server ${catkin_LIBRARIES})
add_dependencies(pertinence_mapping_server miro_teleop_gencpp)

add_executable(spatial_reasoning_server src/spatial_reasoner.cpp)
target_link_libraries(spatial_reasoning_server ${catkin_LIBRARIES})
add_dependencies(spatial_reasoning_server miro_teleop_gencpp)

add_executable(robot_controller src/robot_controller.cpp)
target_link_libraries(robot_controller ${catkin_LIBRARIES})
add_dependencies(robot_controller miro_teleop_gencpp miro_msgs_gencpp miro_msgs_genpy)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
