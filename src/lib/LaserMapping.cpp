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

#include "loam_velodyne/LaserMapping.h"
#include "loam_velodyne/common.h"

namespace loam
{

LaserMapping::LaserMapping(const float& scanPeriod, const size_t& maxIterations)
{
   // initialize mapping odometry and odometry tf messages
   _odomAftMapped.header.frame_id = "/camera_init";
   _odomAftMapped.child_frame_id = "/aft_mapped";

   _aftMappedTrans.frame_id_ = "/camera_init";
   _aftMappedTrans.child_frame_id_ = "/aft_mapped";
}


bool LaserMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
   // fetch laser mapping params
   float fParam;
   int iParam;

   if (privateNode.getParam("scanPeriod", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setScanPeriod(fParam);
         ROS_INFO("Set scanPeriod: %g", fParam);
      }
   }

   if (privateNode.getParam("maxIterations", iParam))
   {
      if (iParam < 1)
      {
         ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
         return false;
      }
      else
      {
         setMaxIterations(iParam);
         ROS_INFO("Set maxIterations: %d", iParam);
      }
   }

   if (privateNode.getParam("deltaTAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaTAbort(fParam);
         ROS_INFO("Set deltaTAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("deltaRAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaRAbort(fParam);
         ROS_INFO("Set deltaRAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("cornerFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid cornerFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterCorner().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set corner down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("surfaceFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid surfaceFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterSurf().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set surface down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("mapFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid mapFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterMap().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set map down size filter leaf size: %g", fParam);
      }
   }

   // advertise laser mapping topics
   _pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
   _pubLaserCloudFullRes  = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);
   _pubOdomAftMapped      = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);

   // subscribe to laser odometry topics
   _subLaserCloudCornerLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_corner_last", 2, &LaserMapping::laserCloudCornerLastHandler, this);

   _subLaserCloudSurfLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_surf_last", 2, &LaserMapping::laserCloudSurfLastHandler, this);

   _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &LaserMapping::laserOdometryHandler, this);

   _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_3", 2, &LaserMapping::laserCloudFullResHandler, this);

   // subscribe to IMU topic
   _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &LaserMapping::imuHandler, this);

   return true;
}



void LaserMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg)
{
   _timeLaserCloudCornerLast = cornerPointsLastMsg->header.stamp;
   laserCloudCornerLast().clear();
   pcl::fromROSMsg(*cornerPointsLastMsg, laserCloudCornerLast());
   _newLaserCloudCornerLast = true;
}

void LaserMapping::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg)
{
   _timeLaserCloudSurfLast = surfacePointsLastMsg->header.stamp;
   laserCloudSurfLast().clear();
   pcl::fromROSMsg(*surfacePointsLastMsg, laserCloudSurfLast());
   _newLaserCloudSurfLast = true;
}

void LaserMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
   _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
   laserCloud().clear();
   pcl::fromROSMsg(*laserCloudFullResMsg, laserCloud());
   _newLaserCloudFullRes = true;
}

void LaserMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   _timeLaserOdometry = laserOdometry->header.stamp;

   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   updateOdometry(-pitch, -yaw, roll,
                  laserOdometry->pose.pose.position.x,
                  laserOdometry->pose.pose.position.y,
                  laserOdometry->pose.pose.position.z);

   _newLaserOdometry = true;
}

void LaserMapping::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   double roll, pitch, yaw;
   tf::Quaternion orientation;
   tf::quaternionMsgToTF(imuIn->orientation, orientation);
   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
   updateIMU({ fromROSTime(imuIn->header.stamp) , roll, pitch });
}

void LaserMapping::spin()
{
   ros::Rate rate(100);
   bool status = ros::ok();

   while (status)
   {
      ros::spinOnce();

      // try processing buffered data
      process();

      status = ros::ok();
      rate.sleep();
   }
}

void LaserMapping::reset()
{
   _newLaserCloudCornerLast = false;
   _newLaserCloudSurfLast = false;
   _newLaserCloudFullRes = false;
   _newLaserOdometry = false;
}

bool LaserMapping::hasNewData()
{
   return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
      _newLaserCloudFullRes && _newLaserOdometry &&
      fabs((_timeLaserCloudCornerLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudSurfLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeLaserOdometry).toSec()) < 0.005;
}

void LaserMapping::process()
{
   if (!hasNewData())// waiting for new data to arrive...
      return;

   reset();// reset flags, etc.

   if (!BasicLaserMapping::process(fromROSTime(_timeLaserOdometry)))
      return;

   publishResult();
}

void LaserMapping::publishResult()
{
   // publish new map cloud according to the input output ratio
   if (hasFreshMap()) // publish new map cloud
      publishCloudMsg(_pubLaserCloudSurround, laserCloudSurroundDS(), _timeLaserOdometry, "/camera_init");

   // publish transformed full resolution input cloud
   publishCloudMsg(_pubLaserCloudFullRes, laserCloud(), _timeLaserOdometry, "/camera_init");

   // publish odometry after mapped transformations
   geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
   (transformAftMapped().rot_z.rad(), -transformAftMapped().rot_x.rad(), -transformAftMapped().rot_y.rad());

   _odomAftMapped.header.stamp = _timeLaserOdometry;
   _odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
   _odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
   _odomAftMapped.pose.pose.orientation.z = geoQuat.x;
   _odomAftMapped.pose.pose.orientation.w = geoQuat.w;
   _odomAftMapped.pose.pose.position.x = transformAftMapped().pos.x();
   _odomAftMapped.pose.pose.position.y = transformAftMapped().pos.y();
   _odomAftMapped.pose.pose.position.z = transformAftMapped().pos.z();
   _odomAftMapped.twist.twist.angular.x = transformBefMapped().rot_x.rad();
   _odomAftMapped.twist.twist.angular.y = transformBefMapped().rot_y.rad();
   _odomAftMapped.twist.twist.angular.z = transformBefMapped().rot_z.rad();
   _odomAftMapped.twist.twist.linear.x = transformBefMapped().pos.x();
   _odomAftMapped.twist.twist.linear.y = transformBefMapped().pos.y();
   _odomAftMapped.twist.twist.linear.z = transformBefMapped().pos.z();
   _pubOdomAftMapped.publish(_odomAftMapped);

   _aftMappedTrans.stamp_ = _timeLaserOdometry;
   _aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
   _aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped().pos.x(),
                                         transformAftMapped().pos.y(),
                                         transformAftMapped().pos.z()));
   _tfBroadcaster.sendTransform(_aftMappedTrans);
}

} // end namespace loam
