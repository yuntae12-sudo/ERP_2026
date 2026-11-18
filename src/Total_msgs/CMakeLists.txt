cmake_minimum_required(VERSION 2.8.3)
project(Total_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY Camera
  FILES
  signalInfo.msg
  trafficInfo.msg
  N_image.msg
  TrafficLight_Arr.msg
  TrafficLightInfo.msg
  TrafficLight.msg
)

add_message_files(
  DIRECTORY ERP
  FILES
  CANFeedBack.msg
  CmdControl.msg
  DriveCmd.msg
  Encoder_vel.msg
  ModeCmd.msg
  SerialFeedBack.msg
)

add_message_files(
  DIRECTORY GPS
  FILES
  GNSSInfo.msg
)

add_message_files(
  DIRECTORY LiDAR
  FILES
  RANSAC.msg
  RANSAC_points.msg
  RANSAC_points_arr.msg
  coeff.msg
  coeff_arr.msg
  fusInfo.msg
  fussInfo.msg
  ground_msg.msg
  ground_msg_arr.msg
  lane.msg
  lane_arr.msg
  objInfo.msg
  objsInfo.msg
  object_msg.msg
  object_msg_arr.msg
  raw4cam.msg
  raw4cam_arr.msg
  tunneloutpoint.msg
  tunneloutpoint_arr.msg
)

add_message_files(
  DIRECTORY Morai
  FILES
  CtrlCmd.msg
  EgoVehicleStatus.msg
  GetTrafficLightStatus.msg
  GPSMessage.msg
)

add_message_files(
  DIRECTORY Planning
  FILES
  totalInfo.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  message_runtime
)

