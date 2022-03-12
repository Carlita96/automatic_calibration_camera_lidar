cmake_minimum_required(VERSION 2.8.3)
project(calibration_camera_lidar)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_ros
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  message_filters
  image_transport
  cv_bridge
  tf
  tf_conversions
  tf2_ros
  tf2_eigen
  message_generation
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
message(STATUS "***** Open CV version: ${OpenCV_VERSION} *****")
####

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  includeTools
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL gazebo_ros
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS includeTools source
)

catkin_install_python(PROGRAMS
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(CALIBRATION_LIBRARY
    source/findSphericalTargetImage.cpp
    source/findSphericalTargetPointCloud.cpp
    includeTools/optimizationProblemTools/ceresSolver3Dto2DTool.cpp
    includeTools/optimizationProblemTools/circleParametersOptimization.cpp
    includeTools/optimizationProblemTools/eigenSolver3Dto3DTool.cpp
    includeTools/optimizationProblemTools/removeOutliersListTool.cpp
    includeTools/optimizationProblemTools/several3Dto3DCalibrationTool.cpp
    includeTools/optimizationProblemTools/several3Dto2DCalibrationTool.cpp
    includeTools/imageTools/generalProcessingTool.cpp
    includeTools/imageTools/projectionTool.cpp
    includeTools/imageTools/smoothEdgeTool.cpp
    includeTools/imageTools/sphericalWarperTool.cpp
    includeTools/pointCloudTools/clusterTool.cpp
    includeTools/pointCloudTools/cropboxTool.cpp
    includeTools/pointCloudTools/normalEstimationTool.cpp
    includeTools/pointCloudTools/planeSegmentationTool.cpp
    includeTools/pointCloudTools/sphereSegmentationTool.cpp
    includeTools/pointCloudTools/cropboxTool.cpp
    includeTools/pointCloudTools/rangeImageTool.cpp
    includeTools/rosTools/publishTools.cpp
    includeTools/eigenTools/rotationMatrixTools.cpp
    includeTools/eigenTools/generalTools.cpp
    )
add_library(${PROJECT_NAME} ${CALIBRATION_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
      ${EIGEN_LIBRARIES} ${pluginlib_DIR} -lstdc++fs )

add_library(animated_sphere_setup_0 source/simulation/animatedSphereSetup0.cpp)
target_link_libraries(animated_sphere_setup_0 ${GAZEBO_LIBRARIES} )

add_library(animated_sphere_setup_1 source/simulation/animatedSphereSetup1.cpp)
target_link_libraries(animated_sphere_setup_1 ${GAZEBO_LIBRARIES} )

add_library(animated_sphere_setup_2 source/simulation/animatedSphereSetup2.cpp)
target_link_libraries(animated_sphere_setup_2 ${GAZEBO_LIBRARIES} )

add_executable(publishCameraInfo source/simulation/publishCameraInfo.cpp)
target_link_libraries(publishCameraInfo ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${pluginlib_DIR} -lstdc++fs)

add_executable(calibrateSensors source/calibrateSensors.cpp)
target_link_libraries(calibrateSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME})

add_executable(checkMiscalibrationSensors source/checkMiscalibrationSensors.cpp)
target_link_libraries(checkMiscalibrationSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME})

add_executable(simulateCalibrateSensors source/simulation/simulateCalibrateSensors.cpp)
target_link_libraries(simulateCalibrateSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME} animated_sphere_setup_0 animated_sphere_setup_1)

add_executable(idealCalibrateSensors source/simulation/idealCalibrateSensors.cpp)
target_link_libraries(idealCalibrateSensors ${catkin_LIBRARIES} ${CALIBRATION_LIB_SOURCES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
    ${pluginlib_DIR}  ${EIGEN_LIBRARIES} -lstdc++fs ${PROJECT_NAME} animated_sphere_setup_0 animated_sphere_setup_1)

