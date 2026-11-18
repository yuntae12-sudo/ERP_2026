cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  Total_msgs
)

add_message_files(
  FILES
  N_image.msg
  signalInfo.msg
  trafficInfo.msg
  TrafficLight.msg
  TrafficLight_Arr.msg
  TrafficLightInfo.msg
  TrafficSign.msg
  TrafficSign_Arr.msg
  TrafficSign_bbox.msg
  object_msg.msg
  object_msg_arr.msg
  LaneInfo.msg
  fussInfo.msg
  fusInfo.msg
  raw4cam_arr.msg
  raw4cam.msg
  MiddleLine.msg
  MiddleLine_arr.msg
  objInfo.msg
  objsInfo.msg
  tunneloutpoint.msg
  coeff.msg
  lane.msg
  DriveCmd.msg
  ModeCmd.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
 CATKIN_DEPENDS roscpp rospy std_msgs Total_msgs message_runtime
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(FILES configuration.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(PROGRAMS
  src/WebcamParser.py
  src/DetectTrafficLight.py
  src/SortTrafficLight.py
  src/DetectTrafficSign.py
  src/SenSorFusion.py
  src/FLIRCamParser.py
  src/BackLaneDetection.py
  src/DetectTrafficLight_ERP.py
  src/OrangeLine_Morai.py
  src/OrangeLine_HSV.py
  src/BackLineDetection_Morai.py
  src/BackLaneDetection.py
  src/SubOrangLine.py
  src/SaveImageFILR.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
