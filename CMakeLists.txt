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
  glow
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GLEW REQUIRED)
find_package(Boost COMPONENTS  serialization system filesystem thread program_options date_time timer chrono regex signals REQUIRED ) 

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs glow
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
  ${GLEW_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../devel/include/gtsam
  ${PROJECT_SOURCE_DIR}/../../devel/include/eigen3
 )

option(ENABLE_NVIDIA_EXT "Enable Nvidia GL capabilites." OFF)
set(OPENGL_VERSION 330 CACHE STRING "Available OpenGL version")

if(ENABLE_NVIDIA_EXT)
  message("Enabling Nvidia OpenGL extensions.")
  add_definitions(-DQUERY_MEMORY_NV)
endif()

add_definitions(${PCL_DEFINITIONS})
add_definitions(-D__GL_VERSION=${OPENGL_VERSION})
message(STATUS "Using OpenGL version ${OPENGL_VERSION}.")


set(COMP_SHADER_SRC ${CMAKE_BINARY_DIR}/computation_shaders.cpp)
set(VIZ_SHADER_SRC ${CMAKE_BINARY_DIR}/visualization_shaders.cpp)

COMPILE_SHADERS(${COMP_SHADER_SRC}
  src/SuMa/shader/color.glsl
  
  src/SuMa/shader/empty.vert
  src/SuMa/shader/empty.frag
  src/SuMa/shader/quad.geom
  src/SuMa/shader/passthrough.frag
  
  src/SuMa/shader/Frame2Model_jacobians.frag
  src/SuMa/shader/Frame2Model_jacobians.geom
  src/SuMa/shader/Frame2Model_jacobians.vert
  
  src/SuMa/shader/render_surfels.frag
  src/SuMa/shader/render_surfels.geom
  src/SuMa/shader/render_surfels.vert
  src/SuMa/shader/render_compose.frag
  src/SuMa/shader/update_surfels.vert
  src/SuMa/shader/update_surfels.geom
  src/SuMa/shader/update_surfels.frag
  src/SuMa/shader/extract_surfels.vert
  src/SuMa/shader/init_radiusConf.vert
  src/SuMa/shader/init_radiusConf.frag
  src/SuMa/shader/copy_surfels.vert
  src/SuMa/shader/copy_surfels.geom
  src/SuMa/shader/gen_surfels.frag
  src/SuMa/shader/gen_surfels.geom
  src/SuMa/shader/gen_surfels.vert
  src/SuMa/shader/gen_surfels330.geom
  src/SuMa/shader/init_depthimg.frag
  src/SuMa/shader/avg_vertexmap.frag
  src/SuMa/shader/gen_depthimg.vert
  src/SuMa/shader/gen_depthimg.frag
  src/SuMa/shader/gen_normalmap.frag
  src/SuMa/shader/gen_vertexmap.frag
  src/SuMa/shader/gen_vertexmap.vert
  src/SuMa/shader/gen_indexmap.vert
  src/SuMa/shader/gen_indexmap.frag
  src/SuMa/shader/bilateral_filter.frag
  
  # needed for surfelmap:
  src/SuMa/shader/draw_surfels.frag
  src/SuMa/shader/draw_surfels.geom
  src/SuMa/shader/draw_surfels.vert
  src/SuMa/shader/draw_surfelPoints.vert
  src/SuMa/shader/draw_submaps.vert
  src/SuMa/shader/draw_submaps.geom
  src/SuMa/shader/coloredvertices.frag
  src/SuMa/shader/coloredvertices.vert
)

COMPILE_SHADERS(${VIZ_SHADER_SRC} 
  src/SuMa/shader/draw_normalmap3d.geom
  src/SuMa/shader/draw_normalmap3d.vert
  src/SuMa/shader/draw_vertexmap3d.vert
  src/SuMa/shader/draw_depthimg.frag
  src/SuMa/shader/draw_normalmap.frag

  src/SuMa/shader/draw_residuals.frag
  src/SuMa/shader/draw_mesh.vert
  src/SuMa/shader/draw_mesh.frag

  
  src/SuMa/shader/draw_posegraph_edge.geom

  src/SuMa/shader/laserscan.vert
  src/SuMa/shader/laserscan.frag
)

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

add_library(robovision
  src/SuMa/rv/CompositeParameter.cpp
  src/SuMa/rv/FileUtil.cpp
  src/SuMa/rv/Laserscan.cpp
  src/SuMa/rv/Math.cpp
  src/SuMa/rv/Parameter.cpp
  src/SuMa/rv/ParameterList.cpp
  src/SuMa/rv/ParameterListIterator.cpp
  src/SuMa/rv/PrimitiveParameters.cpp
  src/SuMa/rv/Random.cpp
  src/SuMa/rv/RangeParameter.cpp
  src/SuMa/rv/string_utils.cpp
  src/SuMa/rv/Stopwatch.cpp
  src/SuMa/rv/transform.cpp
  src/SuMa/rv/XmlDocument.cpp
  src/SuMa/rv/XmlNode.cpp
  src/SuMa/rv/yxml.c
)

add_library(suma
  src/SuMa/core/SurfelMapping.cpp
  src/SuMa/core/Preprocessing.cpp
  src/SuMa/core/Frame2Model.cpp
  src/SuMa/core/SurfelMap.cpp
  src/SuMa/core/lie_algebra.cpp
  src/SuMa/core/LieGaussNewton.cpp
  src/SuMa/core/Posegraph.cpp
  src/SuMa/core/ImagePyramidGenerator.cpp
  
   ${COMP_SHADER_SRC}
)

add_library(utility
  
  src/SuMa/opengl/Mesh.cpp
  src/SuMa/opengl/Model.cpp
  src/SuMa/opengl/ObjReader.cpp

  src/SuMa/util/kitti_utils.cpp
  src/SuMa/util/ScanAccumulator.cpp
  src/SuMa/util/parameters.cpp
  src/SuMa/util/utility.cpp
  src/SuMa/util/parameters.cpp

  ${VIZ_SHADER_SRC}
)

add_library(warp
  src/SuMa/visualizer/preprocessWorker.cpp
)

target_link_libraries(robovision ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  gtsam pthread)
target_link_libraries(suma ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  gtsam pthread robovision)
target_link_libraries(warp suma ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}  gtsam  pthread robovision)

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

add_executable(sumaPreprocess src/SuMa/preprocess_node.cpp)
target_link_libraries(sumaPreprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME} suma robovision utility warp)


