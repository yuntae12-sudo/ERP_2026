cmake_minimum_required(VERSION 3.0.2)
project(GPSIMU_MORAI)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf
  message_generation
  Total_msgs
)

find_package(cmake_modules REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


include_directories(${Eigen_INCLUDE_DIRS} /usr/include/eigen3)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS Eigen
  LIBRARIES INS_Integration_MORAI
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  geometry_msgs
  sensor_msgs
  nav_msgs
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

add_executable(gpsimu src/INS_Kalman_Encoder.cpp)
add_dependencies(gpsimu ${catkin_EXPORTED_TARGETS})
add_compile_options(-w)

target_link_libraries(gpsimu
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

