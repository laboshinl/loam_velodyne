// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <cmath>
#include <vector>

#include <loam_velodyne/common.h>
#include <loam_velodyne/common_ros.h>
#include <loam_velodyne/LoamImu.h>
#include <loam_velodyne/scanRegistrationLib.h>

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ecl/time/stopwatch.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <velodyne_pointcloud/point_types.h>

using std::sin;
using std::cos;
using std::atan2;
using namespace std;

double scanPeriod = 0.1;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubImuTrans;

LoamImuInput imu(scanPeriod);
ScanRegistration scanReg(imu, scanPeriod);

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  imu.handleInput(roll, pitch, yaw,
      imuIn->linear_acceleration.y, imuIn->linear_acceleration.z, imuIn->linear_acceleration.y,
      imuIn->header.stamp.toSec());
}

void processLasserCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &originalLaserCloudIn, ros::Time stamp) {

  double timeScanCur = stamp.toSec();

  ROS_DEBUG_STREAM("[scanRegistration] started processLasserCloud with frame from " << timeScanCur);
  ecl::StopWatch stopWatch;

  ScanRegistration::Outputs outputs;
  scanReg.run(originalLaserCloudIn, timeScanCur, outputs);

  publishCloud(*outputs.laserCloudOut, pubLaserCloud, stamp, "/camera");
  publishCloud(*outputs.cornerPointsSharp, pubCornerPointsSharp, stamp, "/camera");
  publishCloud(*outputs.cornerPointsLessSharp, pubCornerPointsLessSharp, stamp, "/camera");
  publishCloud(*outputs.surfPointsFlat, pubSurfPointsFlat, stamp, "/camera");
  publishCloud(*outputs.surfPointsLessFlat, pubSurfPointsLessFlat, stamp, "/camera");

  sensor_msgs::PointCloud2 imuTransMsg;
  pcl::toROSMsg(imu.to4Points(), imuTransMsg);
  imuTransMsg.header.stamp = stamp;
  imuTransMsg.header.frame_id = "/camera";
  pubImuTrans.publish(imuTransMsg);

  ROS_DEBUG_STREAM("[scanRegistration] processLasserCloud took " << stopWatch.elapsed());
}


void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    }
    return;
  }

  but_velodyne::VelodynePointCloud laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  processLasserCloud(laserCloudIn, laserCloudMsg->header.stamp);
}

bool endsWith(const string &str, const string &suffix) {
  return suffix == str.substr(str.size() - suffix.size());
}

void processCloudFiles(const vector<string> &files) {
  const float delta = 0.1;
  float time = 0;
  for (vector<string>::const_iterator file = files.begin(); file < files.end();
      file++) {
    but_velodyne::VelodynePointCloud cloudIn;

    if (endsWith(*file, ".bin")) {
      but_velodyne::VelodynePointCloud::fromKitti(*file, cloudIn);
      pcl::transformPointCloud(cloudIn, cloudIn,
          cloudIn.getAxisCorrection().inverse().matrix());
    } else {
      pcl::io::loadPCDFile(*file, cloudIn);
    }

    processLasserCloud(cloudIn, ros::Time(time));

    time += delta;
    usleep(90 * 1000);
  }
  cout << "DONE" << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("/velodyne_cloud_2", 2);

  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>
                                        ("/laser_cloud_sharp", 2);

  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_less_sharp", 2);

  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>
                                       ("/laser_cloud_flat", 2);

  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_less_flat", 2);

  pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);

  if (argc > 1) {
    vector<string> files;
    files.assign(argv + 1, argv + argc);
    processCloudFiles(files);
  } else {
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>( "/velodyne_points", 2, laserCloudHandler);
    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imuHandler);
    ros::spin();
  }

  return 0;
}
