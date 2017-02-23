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

#include <loam_velodyne/common.h>
#include <loam_velodyne/common_ros.h>
#include <loam_velodyne/laserOdometryLib.h>

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <ecl/time/stopwatch.hpp>


const float scanPeriod = 0.1;

const int skipFrameNum = 1;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;
double timeImuTrans = 0;

bool newCornerPointsSharp = false;
bool newCornerPointsLessSharp = false;
bool newSurfPointsFlat = false;
bool newSurfPointsLessFlat = false;
bool newLaserCloudFullRes = false;
bool newImuTrans = false;

LaserOdometry::Inputs inputs;
LaserOdometry laserOdom(scanPeriod);

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharp2)
{
  loadCloudFromMsg(cornerPointsSharp2, inputs.cornerPointsSharp, timeCornerPointsSharp);
  newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharp2)
{
  loadCloudFromMsg(cornerPointsLessSharp2, inputs.cornerPointsLessSharp, timeCornerPointsLessSharp);
  newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlat2)
{
  loadCloudFromMsg(surfPointsFlat2, inputs.surfPointsFlat, timeSurfPointsFlat);
  newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlat2)
{
  loadCloudFromMsg(surfPointsLessFlat2, inputs.surfPointsLessFlat, timeSurfPointsLessFlat);
  newSurfPointsLessFlat = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
  loadCloudFromMsg(laserCloudFullRes2, inputs.laserCloudOut, timeLaserCloudFullRes);
  newLaserCloudFullRes = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2)
{
  timeImuTrans = imuTrans2->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ> imuTrans;
  pcl::fromROSMsg(*imuTrans2, imuTrans);

  laserOdom.updateImu(imuTrans);

  newImuTrans = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_sharp", 2, laserCloudSharpHandler);

  ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>
                                             ("/laser_cloud_less_sharp", 2, laserCloudLessSharpHandler);

  ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>
                                      ("/laser_cloud_flat", 2, laserCloudFlatHandler);

  ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);

  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> 
                                         ("/velodyne_cloud_2", 2, laserCloudFullResHandler);

  ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2> 
                                ("/imu_trans", 5, imuTransHandler);

  ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_corner_last", 2);

  ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_surf_last", 2);

  ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> 
                                        ("/velodyne_cloud_3", 2);

  ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";

  int frameCount = skipFrameNum;
  ros::Rate rate(100);
  bool status = ros::ok();
  bool systemInited = false;

  while (status) {
    ros::spinOnce();

    if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat && 
        newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans &&
        fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeImuTrans - timeSurfPointsLessFlat) < 0.005) {

      ROS_DEBUG_STREAM("[laserOdometry] started with frame from " << timeLaserCloudFullRes);
      ecl::StopWatch stopWatch;

      newCornerPointsSharp = false;
      newCornerPointsLessSharp = false;
      newSurfPointsFlat = false;
      newSurfPointsLessFlat = false;
      newLaserCloudFullRes = false;
      newImuTrans = false;

      if (!systemInited) {
        publishCloud(*inputs.cornerPointsLessSharp, pubLaserCloudCornerLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
        publishCloud(*inputs.surfPointsLessFlat, pubLaserCloudSurfLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
      }

      LaserOdometry::Outputs outputs;
      laserOdom.run(inputs, outputs);

      if(systemInited) {
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(outputs.t[2], -outputs.t[0], -outputs.t[1]);

        laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserOdometry.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry.pose.pose.orientation.z = geoQuat.x;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = outputs.t[3];
        laserOdometry.pose.pose.position.y = outputs.t[4];
        laserOdometry.pose.pose.position.z = outputs.t[5];
        pubLaserOdometry.publish(laserOdometry);

        laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans.setOrigin(tf::Vector3(outputs.t[3], outputs.t[4], outputs.t[5]));
        tfBroadcaster.sendTransform(laserOdometryTrans);

        frameCount++;
        if (frameCount > skipFrameNum) {
          frameCount = 0;
          publishCloud(*outputs.corners, pubLaserCloudCornerLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
          publishCloud(*outputs.surfels, pubLaserCloudSurfLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
          publishCloud(*outputs.cloud, pubLaserCloudFullRes, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
        }
      }
      systemInited = true;

      ROS_DEBUG_STREAM("[laserOdometry] took " << stopWatch.elapsed());
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
