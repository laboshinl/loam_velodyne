cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)

FIND_PACKAGE(Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

find_package(but_velodyne REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${ButVELODYNE_INCLUDE_DIRS} ../include)
link_directories( ${PCL_LIBRARY_DIRS} ${ButVELODYNE_LIBRARY_DIRS})
add_definitions(-DVELODYNE_MODEL=64)

add_executable(loam-odom loam-odom.cpp ../src/scanRegistrationLib.cpp ../src/laserOdometryLib.cpp
../src/common.cpp ../src/build_transform.cpp ../src/laserMappingLib.cpp)
target_link_libraries(loam-odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ButVELODYNE_LIBRARIES} ${Boost_LIBRARIES})
