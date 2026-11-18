cmake_minimum_required(VERSION 3.0.2)
project(INS_Integration)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  Total_msgs
)

find_package(cmake_modules REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg

  FILES
  # totalInfo.msg
  GNSSInfo.msg
  Encoder_vel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS Eigen
  LIBRARIES INS_Integration
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  sensor_msgs
  message_runtime
  message_generation
  Total_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(erpgps src/INS_Kalman_Encoder.cpp)
add_dependencies(erpgps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(erpgps
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(sig src/signal_test.cpp)

target_link_libraries(sig
  ${catkin_LIBRARIES}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

