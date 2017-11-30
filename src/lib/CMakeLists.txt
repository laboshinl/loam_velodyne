add_library(loam
            math_utils.h
            CircularBuffer.h
            ScanRegistration.h
            ScanRegistration.cpp
            VelodyneScanRegistration.h
            VelodyneScanRegistration.cpp)
target_link_libraries(loam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
