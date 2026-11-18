################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(erp42_serial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  ## erp42_msgs
  Total_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_srvs std_msgs Total_msgs
)

################################################################################
# Build
################################################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(erp42_safety src/safety_control.cpp src/SerialPort/serial_port.cpp)

add_executable(erp42_serial_node
    src/erp42_serial.cpp
    src/erp42_serial_node.cpp
    )
add_dependencies(erp42_serial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(erp42_serial_node erp42_safety ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################


################################################################################
# Test
################################################################################
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(erp42_safety_test test/safety_test.cpp)
  if(TARGET erp42_safety_test)
    target_link_libraries(erp42_safety_test erp42_safety util pthread)
  endif()
endif()
