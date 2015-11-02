cmake_minimum_required(VERSION 2.8.3)
project(online_stereo_calibration_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  image_transport
  #rviz_visual_tools
  tf_conversions
  nodelet
)

find_package(OpenCV REQUIRED)
#find_package(CUDA)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
#   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(lib/stereo_calib_lib/include)

## stereo calibration lib
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/stereo_calib_lib)

####################
# NODE EXECUTABLES #
####################

# Declare a cpp executable
add_executable(stereo_calibration_node src/StereoCalibrationRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(stereo_calibration_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   STEREO_CALIB_LIB
   ${OpenCV_LIBRARIES}
)

## Declare a cpp executable
add_executable(perfect_stereo_calibration_node src/PerfectStereoCalibrationRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(perfect_stereo_calibration_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
)


#####################
# NODELET LIBRARIES #
#####################

# Nodelet library
add_library(perfect_stereo_calibration_nodelet src/PerfectStereoCalibrationRos.cpp src/nodelets/PerfectStereoCalibrationRosNodelet.cpp)
target_link_libraries(perfect_stereo_calibration_nodelet
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES})

install(TARGETS perfect_stereo_calibration_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_library(stereo_calibration_nodelet src/StereoCalibrationRos.cpp src/nodelets/StereoCalibrationRosNodelet.cpp)
target_link_libraries(stereo_calibration_nodelet
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES})

install(TARGETS stereo_calibration_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES online_stereo_calibration_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





