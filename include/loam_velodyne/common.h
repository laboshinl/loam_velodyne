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

#ifndef __LOAM_COMMON_H__
#define __LOAM_COMMON_H__


#include <cmath>

#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <velodyne_pointcloud/point_types.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

void transformAssociateToMap(const std::vector<float> &beforeMapping,
                             const std::vector<float> &afterMapping,
                             const std::vector<float> &current,
                             std::vector<float> &output);

void loadCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr out_cloud,
    double &out_time);

template <typename PointT>
inline float pointsSqDistance(const PointT &pt1, const PointT &pt2) {
  float dx = pt1.x - pt2.x;
  float dy = pt1.y - pt2.y;
  float dz = pt1.z - pt2.z;
  return dx*dx + dy*dy + dz*dz;
}

template <typename PointT>
inline float pointsDistance(const PointT &pt1, const PointT &pt2) {
  return sqrt(pointsSqDistance(pt1, pt2));
}

template <typename PointT>
inline float normalizedPointsSqDistance(const PointT &pt1, float normalization1,
    const PointT &pt2, float normalization2) {
  float dx = pt1.x*normalization1 - pt2.x*normalization2;
  float dy = pt1.y*normalization1 - pt2.y*normalization2;
  float dz = pt1.z*normalization1 - pt2.z*normalization2;
  return dx*dx + dy*dy + dz*dz;
}

template <typename PointT>
inline float normalizedPointsDistance(const PointT &pt1, float normalization1,
    const PointT &pt2, float normalization2) {
  return sqrt(normalizedPointsSqDistance(pt1, normalization1, pt2, normalization2));
}

template <typename PointT>
inline float pointSqNorm(const PointT &pt) {
  return pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
}

template <typename PointT>
inline float pointNorm(const PointT &pt) {
  return sqrt(pointSqNorm(pt));
}

template <typename PointT>
void publishCloud(const pcl::PointCloud<PointT> &cloud, ros::Publisher &publisher, ros::Time stamp, std::string frame_id) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  publisher.publish(msg);
}

#endif
