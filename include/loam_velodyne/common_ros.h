#ifndef __LOAM_COMMON_ROS_H__
#define __LOAM_COMMON_ROS_H__

#include <loam_velodyne/common.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

void loadCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr out_cloud,
    double &out_time);

template <typename PointT>
void publishCloud(const pcl::PointCloud<PointT> &cloud, ros::Publisher &publisher, ros::Time stamp, std::string frame_id) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  publisher.publish(msg);
}

#endif
