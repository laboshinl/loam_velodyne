Installation:

```
cd /opt/ros/indigo/stacks/
git clone https://github.com/laboshinl/loam_velodyne.git
cd loam_velodyne
cmake .
rosmake
```

Running:
```
roslaunch loam_velodyne.launch
```

In second terminal play sample velodyne data from rosbag (http://www.frc.ri.cmu.edu/~jizhang03/Datasets/nsh_indoor_outdoor.bag):
```
rosbag play ~/Downloads/nsh_indoor_outdoor.bag 
```

Or read from velodyne pcap (http://midas3.kitware.com/midas/download/item/215411):
```
roslaunch velodyne_pointcloud 32e_points.launch calibration:=/home/laboshinl/calib.yaml pcap:="/home/laboshinl/Downloads/velodyne.pcap"
```

