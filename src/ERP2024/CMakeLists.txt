cmake_minimum_required(VERSION 3.0.2)
project(ERP2024)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  visualization_msgs
  roscpp
  std_msgs
  rospy
  Total_msgs
  camera
  roslib
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
 # INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS system_lib Eigen3
 Total_msgs
 camera
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(cmake_modules REQUIRED)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(myHeader 
  src/Control/Control.cpp 
  src/Global/Global.cpp 
  src/Parser/Parser.cpp 
  src/Planning/Planning.cpp 
  src/Planning/Delivery.cpp
  src/Planning/Delivery_LiDAR.cpp
  # src/Planning/Delivery_RefPath.cpp
  src/Planning/Avoid.cpp
  src/Planning/Avoid_Lidar.cpp
  src/Planning/Parking.cpp
  src/Planning/Uturn.cpp
  src/Planning/Uturn_LiDAR.cpp
  src/Planning/Uturn_Dis.cpp
  src/Planning/Tunnel.cpp
  src/Planning/Narrow.cpp 
  src/Planning/Visualize.cpp 
  src/MapRecord_erp.cpp
)

target_link_libraries(myHeader ${catkin_LIBRARIES})

add_executable(plnctrl src/Main.cpp)
target_link_libraries(plnctrl myHeader
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  yaml-cpp
)


# add_executable(record src/MapRecord_erp.cpp)
# target_link_libraries(record myHeader
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
# )

add_dependencies(myHeader ${catkin_EXPORTED_TARGETS})
add_dependencies(plnctrl ${catkin_EXPORTED_TARGETS})