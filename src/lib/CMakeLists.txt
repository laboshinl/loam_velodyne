add_library(loam
            math_utils.h
            ScanRegistration.cpp
            BasicScanRegistration.cpp
            MultiScanRegistration.cpp
            LaserOdometry.cpp
            BasicLaserOdometry.cpp
            LaserMapping.cpp
            BasicLaserMapping.cpp
            TransformMaintenance.cpp
            BasicTransformMaintenance.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
