cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS  serialization system filesystem thread program_options date_time timer chrono regex signals REQUIRED ) 

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Boost
  INCLUDE_DIRS include
  LIBRARIES aloam
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../devel/include/gtsam
  ${PROJECT_SOURCE_DIR}/../../devel/include/eigen3
 )

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
src/aloam/points_segmentation/projection_params.cpp
src/aloam/points_segmentation/scan_projection.cpp
src/aloam/points_segmentation/rich_point.cpp
src/aloam/points_segmentation/feature_extractor/depth_ground_remover.cpp
src/aloam/points_segmentation/feature_extractor/feature_extractor.cpp
src/aloam/points_segmentation/image_labelers/abstract_image_labeler.cpp
src/aloam/points_segmentation/image_labelers/diff_helpers/angle_diff.cpp
src/aloam/points_segmentation/image_labelers/diff_helpers/line_dist_diff.cpp
)

target_link_libraries(${PROJECT_NAME}
${Boost_LIBRARIES} -lgtsam -ltbb
/usr/lib/x86_64-linux-gnu/libtbb.so
)

add_executable(ascanRegistration src/aloam/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(alaserOdometry src/aloam/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserFactorGraphOdometry src/aloam/laserFactorGraphOdometry.cpp)
target_link_libraries(alaserFactorGraphOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam)

add_executable(alaserMapping src/aloam/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserFactorGraphMapping src/aloam/laserFactorGraphMapping.cpp)
target_link_libraries(alaserFactorGraphMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam)

add_executable(kittiHelper src/aloam/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
