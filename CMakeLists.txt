cmake_minimum_required(VERSION 3.0.2)
project(synexens_ros1 LANGUAGES C CXX)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  image_geometry
  nodelet
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nodelet
  cv_bridge
  camera_info_manager
)

find_package(OpenCV REQUIRED)

catkin_package()

############################
#### Synexens SDK 4.0 ######
############################
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_videoio opencv_calib3d)
set(SY_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk)
set(SY_INCLUDE_DIR ${SY_SDK_PATH}/include)
set(SY_SDK_LIBS_DIR ${SY_SDK_PATH}/lib)
set(SY_SDK_LIBS SynexensSDK csreconstruction2.0 SonixCamera ${OpenCV_LIBS})

set(OPENCV_LIB_DIR ${SY_SDK_PATH}/opencv)

link_directories(${SY_SDK_LIBS_DIR} ${OPENCV_LIB_DIR})

add_executable(${PROJECT_NAME}_node src/SYRosNode.cpp src/SYRosDevice.cpp src/SYCalibrationTransformData.cpp src/SYRosDeviceParams.cpp)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SY_INCLUDE_DIR}
  "include"
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${SY_SDK_LIBS}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  launch
  rviz
  scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  ${SYN_LIB_DIR}/
  ${OPENCV_LIB_DIR}/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)