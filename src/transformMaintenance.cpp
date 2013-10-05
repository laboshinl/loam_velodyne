#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

double timeOdomBefMapped;
double timeOdomAftMapped;

float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

ros::Publisher *pubLaserOdometry2Pointer = NULL;
tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
nav_msgs::Odometry laserOdometry2;
tf::StampedTransform laserOdometryTrans2;

void transformAssociateToMap()
{
  float x1 = cos(transformSum[2]) * (transformBefMapped[3] - transformSum[3]) 
           + sin(transformSum[2]) * (transformBefMapped[4] - transformSum[4]);
  float y1 = -sin(transformSum[2]) * (transformBefMapped[3] - transformSum[3])
           + cos(transformSum[2]) * (transformBefMapped[4] - transformSum[4]);
  float z1 = transformBefMapped[5] - transformSum[5];

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[1]) * x1 - sin(transformSum[1]) * z1;
  transformIncre[4] = y2;
  transformIncre[5] = sin(transformSum[1]) * x1 + cos(transformSum[1]) * z1;

  transformIncre[0] = transformBefMapped[0] - transformSum[0];
  transformIncre[1] = transformBefMapped[1] - transformSum[1];
  transformIncre[2] = transformBefMapped[2] - transformSum[2];

  transformMapped[0] = transformAftMapped[0] - transformIncre[0];
  transformMapped[1] = transformAftMapped[1] - transformIncre[1];
  transformMapped[2] = transformAftMapped[2] - transformIncre[2];

  x1 = cos(transformMapped[1]) * transformIncre[3] 
     + sin(transformMapped[1]) * transformIncre[5];
  y1 = transformIncre[4];
  z1 = -sin(transformMapped[1]) * transformIncre[3] 
     + cos(transformMapped[1]) * transformIncre[5];

  x2 = x1;
  y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
  z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

  transformMapped[3] = transformAftMapped[3] 
                         - (cos(transformMapped[2]) * x2 - sin(transformMapped[2]) * y2);
  transformMapped[4] = transformAftMapped[4] 
                         - (sin(transformMapped[2]) * x2 + cos(transformMapped[2]) * y2);
  transformMapped[5] = transformAftMapped[5] - z2;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  if (fabs(timeOdomBefMapped - timeOdomAftMapped) < 0.005) {

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;

    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;

    transformAssociateToMap();

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw
              (transformMapped[2], -transformMapped[0], -transformMapped[1]);

    laserOdometry2.header.stamp = laserOdometry->header.stamp;
    laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
    laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
    laserOdometry2.pose.pose.orientation.z = geoQuat.x;
    laserOdometry2.pose.pose.orientation.w = geoQuat.w;
    laserOdometry2.pose.pose.position.x = transformMapped[3];
    laserOdometry2.pose.pose.position.y = transformMapped[4];
    laserOdometry2.pose.pose.position.z = transformMapped[5];
    pubLaserOdometry2Pointer->publish(laserOdometry2);

    laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
    laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
    tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);
  }
}

void odomBefMappedHandler(const nav_msgs::Odometry::ConstPtr& odomBefMapped)
{
  timeOdomBefMapped = odomBefMapped->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomBefMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformBefMapped[0] = -pitch;
  transformBefMapped[1] = -yaw;
  transformBefMapped[2] = roll;

  transformBefMapped[3] = odomBefMapped->pose.pose.position.x;
  transformBefMapped[4] = odomBefMapped->pose.pose.position.y;
  transformBefMapped[5] = odomBefMapped->pose.pose.position.z;
}

void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
  timeOdomAftMapped = odomAftMapped->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle nh;

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
                                     ("/cam_to_init", 1, laserOdometryHandler);

  ros::Subscriber subOdomBefMapped = nh.subscribe<nav_msgs::Odometry> 
                                     ("/bef_mapped_to_init_2", 1, odomBefMappedHandler);

  ros::Subscriber subOdomAftMapped = nh.subscribe<nav_msgs::Odometry> 
                                     ("/aft_mapped_to_init_2", 1, odomAftMappedHandler);

  ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/cam_to_init_2", 1);
  pubLaserOdometry2Pointer = &pubLaserOdometry2;
  laserOdometry2.header.frame_id = "/camera_init_2";
  laserOdometry2.child_frame_id = "/camera";

  tf::TransformBroadcaster tfBroadcaster2;
  tfBroadcaster2Pointer = &tfBroadcaster2;
  laserOdometryTrans2.frame_id_ = "/camera_init_2";
  laserOdometryTrans2.child_frame_id_ = "/camera";

  ros::spin();

  return 0;
}
