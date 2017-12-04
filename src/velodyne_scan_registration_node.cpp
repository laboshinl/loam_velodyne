#include <ros/ros.h>
#include "loam_velodyne/VelodyneScanRegistration.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::VelodyneScanRegistration veloScan(0.1, 16);

  if (veloScan.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
