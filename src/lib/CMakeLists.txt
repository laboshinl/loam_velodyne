add_library(loam
            math_utils.h
            CircularBuffer.h
            ScanRegistration.h
            ScanRegistration.cpp
            VelodyneScanRegistration.h
            VelodyneScanRegistration.cpp
            LaserOdometry.h
            LaserOdometry.cpp
            LaserMapping.h
            LaserMapping.cpp
            TransformMaintenance.h
            TransformMaintenance.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
