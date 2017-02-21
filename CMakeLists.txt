cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  )

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
#	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

find_package(but_velodyne REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${ButVELODYNE_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS} ${ButVELODYNE_LIBRARY_DIRS})
add_definitions(-DVELODYNE_MODEL=64)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ButVELODYNE_LIBRARIES})

add_executable(laserOdometry src/laserOdometry.cpp src/common.cpp src/build_transform.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ButVELODYNE_LIBRARIES})

add_executable(laserMapping src/laserMapping.cpp src/common.cpp src/build_transform.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ButVELODYNE_LIBRARIES})

add_executable(transformMaintenance src/transformMaintenance.cpp src/common.cpp src/build_transform.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ButVELODYNE_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # TODO: Download test data
  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
    MD5 3d5194e6981975588b7a93caebf79ba4)
  add_custom_target(${PROJECT_NAME}_test_data
    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
  configure_file(tests/loam.test.in
                 ${PROJECT_BINARY_DIR}/test/loam.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
    DEPENDENCIES
      ${PROJECT_NAME}_test_data
      scanRegistration
      laserOdometry
      laserMapping
      transformMaintenance)
endif()


