cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories( 
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR} 
	${PCL_INCLUDE_DIRS})

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   nav_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)


add_executable(scanRegistration src/scanRegistration.cpp)
add_executable(laserOdometry src/laserOdometry.cpp)
add_executable(laserMapping src/laserMapping.cpp)
add_executable(transformMaintenance src/transformMaintenance.cpp)

target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

