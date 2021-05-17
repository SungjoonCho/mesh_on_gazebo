cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_plugin_pc)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  std_msgs 
  roscpp 
  pcl_conversions 
  pcl_ros
  sensor_msgs
  shape_msgs
  pcl_msgs
  visualization_msgs
  image_transport
  image_geometry
  depth_image_proc
  tf
  tf_conversions
)
#find_package(ignition-transport4 REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  #${std_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES gazebo_plugin_pc
#   CATKIN_DEPENDS pcl_ros roscpp sensor_msgs shape_msgs visualization_msgs pcl_msgs image_transport image_geometry depth_image_proc tf tf_conversions
#   DEPENDS system_lib
# )

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )


# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(realsense_depth_to_mesh SHARED MySrc/realsense_depth_to_mesh.cpp)
add_library(PointcloudGazebo_pcd_string SHARED MySrc/PointcloudGazebo_pcd_string.cpp)
add_library(PointcloudGazebo_pcd_publish SHARED MySrc/PointcloudGazebo_pcd_publish.cpp)
add_library(PointcloudGazebo_pcd_mesh SHARED MySrc/PointcloudGazebo_pcd_mesh.cpp)


target_link_libraries(realsense_depth_to_mesh
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${realsense2_LIBRARY}
  ${OpenCV_LIBRARIES} 
)
target_link_libraries(PointcloudGazebo_pcd_string
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(PointcloudGazebo_pcd_publish
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(PointcloudGazebo_pcd_mesh
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)


# if (${gazebo_VERSION_MAJOR} LESS 6)
#   include(FindBoost)
#   find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
#   target_link_libraries(PointcloudGazebo_pcd_string ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# else()
#   target_link_libraries(PointcloudGazebo_pcd_string ${GAZEBO_LIBRARIES})
# endif()