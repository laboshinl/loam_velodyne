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

#include <math.h>

#include "loam_velodyne/common.h"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include "loam_velodyne/nanoflann_pcl.h"
#include "math_utils.h"

const float scanPeriod = 0.1;

const int stackFrameNum = 1;
const int mapFrameNum = 5;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth = 10;
const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth = 21;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

Twist transformSum;
Twist transformIncre;
Twist transformTobeMapped;
Twist transformBefMapped;
Twist transformAftMapped;

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};

void transformAssociateToMap()
{
  Vector3 v0 = transformBefMapped.pos -  transformSum.pos;
  Vector3 v1 = rotateY( v0, -(transformSum.rot_y) );
  Vector3 v2 = rotateX( v1, -(transformSum.rot_x) );
  transformIncre.pos = rotateZ( v2, -(transformSum.rot_z) );

  float sbcx = transformSum.rot_x.sin();
  float cbcx = transformSum.rot_x.cos();
  float sbcy = transformSum.rot_y.sin();
  float cbcy = transformSum.rot_y.cos();
  float sbcz = transformSum.rot_z.sin();
  float cbcz = transformSum.rot_z.cos();

  float sblx = transformBefMapped.rot_x.sin();
  float cblx = transformBefMapped.rot_x.cos();
  float sbly = transformBefMapped.rot_y.sin();
  float cbly = transformBefMapped.rot_y.cos();
  float sblz = transformBefMapped.rot_z.sin();
  float cblz = transformBefMapped.rot_z.cos();

  float salx = transformAftMapped.rot_x.sin();
  float calx = transformAftMapped.rot_x.cos();
  float saly = transformAftMapped.rot_y.sin();
  float caly = transformAftMapped.rot_y.cos();
  float salz = transformAftMapped.rot_z.sin();
  float calz = transformAftMapped.rot_z.cos();

  float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
            - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
            - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
            - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
            - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
  transformTobeMapped.rot_x = -asin(srx);

  float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
               - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
               - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
               + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
               + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
  float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
               - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
               + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
               + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
               - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
               + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
  transformTobeMapped.rot_y = atan2(srycrx / transformTobeMapped.rot_x.cos(),
                                 crycrx / transformTobeMapped.rot_x.cos());
  
  float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
               - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
               - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
               + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
               - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
               - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
               + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  transformTobeMapped.rot_z = atan2(srzcrx / transformTobeMapped.rot_x.cos(),
                                 crzcrx / transformTobeMapped.rot_x.cos());

  Vector3 v3;
  v1 = rotateZ( transformIncre.pos,  transformTobeMapped.rot_z);
  v2 = rotateX( v1,  transformTobeMapped.rot_x);
  v3 = rotateY( v2,  transformTobeMapped.rot_y);
  transformTobeMapped.pos = transformAftMapped.pos - v3;
}

void transformUpdate()
{
  if (imuPointerLast >= 0) {
    float imuRollLast = 0, imuPitchLast = 0;
    while (imuPointerFront != imuPointerLast) {
      if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
      imuRollLast = imuRoll[imuPointerFront];
      imuPitchLast = imuPitch[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
    }

    transformTobeMapped.rot_x = 0.998 * transformTobeMapped.rot_x.value() + 0.002 * imuPitchLast;
    transformTobeMapped.rot_z = 0.998 * transformTobeMapped.rot_z.value() + 0.002 * imuRollLast;
  }

  transformBefMapped = transformSum;
  transformAftMapped = transformTobeMapped;
}

void pointAssociateToMap(PointType const * const pi, PointType * const po)
{
  Vector3 v1 = rotateZ( *pi, transformTobeMapped.rot_z);
  Vector3 v2 = rotateX(  v1, transformTobeMapped.rot_x);
  Vector3 v3 = rotateY(  v2, transformTobeMapped.rot_y);
  v3 += transformTobeMapped.pos;

  po->x = v3.x();
  po->y = v3.y();
  po->z = v3.z();
  po->intensity = pi->intensity;
}

void pointAssociateTobeMapped(PointType const * const pi, PointType * const po)
{
  Vector3 v0 = Vector3(*pi) - transformTobeMapped.pos;
  Vector3 v1 = rotateY( v0, -transformTobeMapped.rot_y);
  Vector3 v2 = rotateX( v1, -transformTobeMapped.rot_x);
  Vector3 v3 = rotateZ( v2, -transformTobeMapped.rot_z);

  po->x = v3.x();
  po->y = v3.y();
  po->z = v3.z();
  po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
  timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

  laserCloudCornerLast->clear();
  pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

  newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
  timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

  laserCloudSurfLast->clear();
  pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

  newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
  timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

  laserCloudFullRes->clear();
  pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

  newLaserCloudFullRes = true;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  timeLaserOdometry = laserOdometry->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformSum.rot_x = -pitch;
  transformSum.rot_y = -yaw;
  transformSum.rot_z = roll;

  transformSum.pos.x() = laserOdometry->pose.pose.position.x;
  transformSum.pos.y() = laserOdometry->pose.pose.position.y;
  transformSum.pos.z() = laserOdometry->pose.pose.position.z;

  newLaserOdometry = true;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);

  ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler);

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
                                     ("/laser_odom_to_init", 5, laserOdometryHandler);

  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> 
                                         ("/velodyne_cloud_3", 2, laserCloudFullResHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_surround", 1);

  ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> 
                                        ("/velodyne_cloud_registered", 2);

  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.child_frame_id = "/aft_mapped";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform aftMappedTrans;
  aftMappedTrans.frame_id_ = "/camera_init";
  aftMappedTrans.child_frame_id_ = "/aft_mapped";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  PointType pointOri, pointSel, pointProj, coeff;

  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;
  Eigen::Matrix3f matA1;
  Eigen::Matrix<float, 1, 3> matD1;
  Eigen::Matrix3f matV1;

  matA0.setZero();
  matB0.setConstant(-1);
  matX0.setZero();

  matA1.setZero();
  matD1.setZero();
  matV1.setZero();

  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

  pcl::VoxelGrid<PointType> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
  }

  int frameCount = stackFrameNum - 1;
  int mapFrameCount = mapFrameNum - 1;
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
        fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005) {
      newLaserCloudCornerLast = false;
      newLaserCloudSurfLast = false;
      newLaserCloudFullRes = false;
      newLaserOdometry = false;

      frameCount++;
      if (frameCount >= stackFrameNum) {
        transformAssociateToMap();

        int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        for (int i = 0; i < laserCloudCornerLastNum; i++) {
          pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
          laserCloudCornerStack2->push_back(pointSel);
        }

        int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
        for (int i = 0; i < laserCloudSurfLastNum; i++) {
          pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
          laserCloudSurfStack2->push_back(pointSel);
        }
      }

      if (frameCount >= stackFrameNum) {
        frameCount = 0;

        PointType pointOnYAxis;
        pointOnYAxis.x = 0.0;
        pointOnYAxis.y = 10.0;
        pointOnYAxis.z = 0.0;
        pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

        int centerCubeI = int((transformTobeMapped.pos.x() + 25.0) / 50.0) + laserCloudCenWidth;
        int centerCubeJ = int((transformTobeMapped.pos.y() + 25.0) / 50.0) + laserCloudCenHeight;
        int centerCubeK = int((transformTobeMapped.pos.z() + 25.0) / 50.0) + laserCloudCenDepth;

        if (transformTobeMapped.pos.x() + 25.0 < 0) centerCubeI--;
        if (transformTobeMapped.pos.y() + 25.0 < 0) centerCubeJ--;
        if (transformTobeMapped.pos.z() + 25.0 < 0) centerCubeK--;

        while (centerCubeI < 3) {
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = laserCloudWidth - 1;
              pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; i >= 1; i--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI++;
          laserCloudCenWidth++;
        }

        while (centerCubeI >= laserCloudWidth - 3) {
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = 0;
              pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; i < laserCloudWidth - 1; i++) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI--;
          laserCloudCenWidth--;
        }

        while (centerCubeJ < 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int j = laserCloudHeight - 1;
              pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; j >= 1; j--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }
 
          centerCubeJ++;
          laserCloudCenHeight++;
        } 

        while (centerCubeJ >= laserCloudHeight - 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int j = 0;
              pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; j < laserCloudHeight - 1; j++) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeJ--;
          laserCloudCenHeight--;
        }

        while (centerCubeK < 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
              int k = laserCloudDepth - 1;
              pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; k >= 1; k--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK++;
          laserCloudCenDepth++;
        }
      
        while (centerCubeK >= laserCloudDepth - 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
              int k = 0;
              pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; k < laserCloudDepth - 1; k++) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK--;
          laserCloudCenDepth--;
        }

        int laserCloudValidNum = 0;
        int laserCloudSurroundNum = 0;
        for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
          for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
            for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
              if (i >= 0 && i < laserCloudWidth && 
                  j >= 0 && j < laserCloudHeight && 
                  k >= 0 && k < laserCloudDepth) {

                float centerX = 50.0 * (i - laserCloudCenWidth);
                float centerY = 50.0 * (j - laserCloudCenHeight);
                float centerZ = 50.0 * (k - laserCloudCenDepth);

                PointType transform_pos = (pcl::PointXYZI)transformTobeMapped.pos;

                bool isInLaserFOV = false;
                for (int ii = -1; ii <= 1; ii += 2) {
                  for (int jj = -1; jj <= 1; jj += 2) {
                    for (int kk = -1; kk <= 1; kk += 2) {
                      PointType corner;
                      corner.x = centerX + 25.0 * ii;
                      corner.y = centerY + 25.0 * jj;
                      corner.z = centerZ + 25.0 * kk;

                      float squaredSide1 = calcSquaredDiff(transform_pos, corner);
                      float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner);

                      float check1 = 100.0 + squaredSide1 - squaredSide2
                                   - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                      float check2 = 100.0 + squaredSide1 - squaredSide2
                                   + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                      if (check1 < 0 && check2 > 0) {
                        isInLaserFOV = true;
                      }
                    }
                  }
                }

                if (isInLaserFOV) {
                  laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j 
                                                       + laserCloudWidth * laserCloudHeight * k;
                  laserCloudValidNum++;
                }
                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j 
                                                             + laserCloudWidth * laserCloudHeight * k;
                laserCloudSurroundNum++;
              }
            }
          }
        }

        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < laserCloudValidNum; i++) {
          *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
          *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }
        int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

        int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
        for (int i = 0; i < laserCloudCornerStackNum2; i++) {
          pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
        }

        int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
        for (int i = 0; i < laserCloudSurfStackNum2; i++) {
          pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
        }

        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        laserCloudCornerStack2->clear();
        laserCloudSurfStack2->clear();

        if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {    

         nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
         nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

         kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMap);
         kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMap);

          pointSearchInd.resize(5);
          pointSearchSqDis.resize(5);

          for (int iterCount = 0; iterCount < 10; iterCount++) {
            laserCloudOri->clear();
            coeffSel->clear();

            for (int i = 0; i < laserCloudCornerStackNum; i++) {
              pointOri = laserCloudCornerStack->points[i];
              pointAssociateToMap(&pointOri, &pointSel);
              kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis );
              
              if (pointSearchSqDis[4] < 1.0) {
                Vector3 vc(0,0,0);

                for (int j = 0; j < 5; j++) {
                  vc.x() += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                  vc.y() += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                  vc.z() += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                }
                vc /= 5.0;

                Eigen::Matrix3f mat_a;
                mat_a.setZero();

                for (int j = 0; j < 5; j++) {
                  float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - vc.x();
                  float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - vc.y();
                  float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - vc.z();

                  mat_a(0,0) += ax * ax;
                  mat_a(0,1) += ax * ay;
                  mat_a(0,2) += ax * az;
                  mat_a(1,1) += ay * ay;
                  mat_a(1,2) += ay * az;
                  mat_a(2,2) += az * az;
                }
                matA1 = mat_a / 5.0;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
                matD1 = esolver.eigenvalues().real();
                matV1 = esolver.eigenvectors().real();

                if (matD1(0, 0) > 3 * matD1(0, 1)) {

                  float x0 = pointSel.x;
                  float y0 = pointSel.y;
                  float z0 = pointSel.z;
                  float x1 = vc.x() + 0.1 * matV1(0, 0);
                  float y1 = vc.y() + 0.1 * matV1(0, 1);
                  float z1 = vc.z() + 0.1 * matV1(0, 2);
                  float x2 = vc.x() - 0.1 * matV1(0, 0);
                  float y2 = vc.y() - 0.1 * matV1(0, 1);
                  float z2 = vc.z() - 0.1 * matV1(0, 2);

                  float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                             * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                             + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                             * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                             + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                             * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                  float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                  float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                  float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                           - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                  float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                           + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                  float ld2 = a012 / l12;

                  pointProj = pointSel;
                  pointProj.x -= la * ld2;
                  pointProj.y -= lb * ld2;
                  pointProj.z -= lc * ld2;

                  float s = 1 - 0.9 * fabs(ld2);

                  coeff.x = s * la;
                  coeff.y = s * lb;
                  coeff.z = s * lc;
                  coeff.intensity = s * ld2;

                  if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    coeffSel->push_back(coeff);
                  }
                }
              }
            }

            for (int i = 0; i < laserCloudSurfStackNum; i++) {
              pointOri = laserCloudSurfStack->points[i];
              pointAssociateToMap(&pointOri, &pointSel); 
              kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis );

              if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                  matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                  matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                  matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                }
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                  if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                      pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                      pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                  }
                }

                if (planeValid) {
                  float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                  pointProj = pointSel;
                  pointProj.x -= pa * pd2;
                  pointProj.y -= pb * pd2;
                  pointProj.z -= pc * pd2;

                  float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                          + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                  coeff.x = s * pa;
                  coeff.y = s * pb;
                  coeff.z = s * pc;
                  coeff.intensity = s * pd2;

                  if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    coeffSel->push_back(coeff);
                  }
                }
              }
            }

            float srx = transformTobeMapped.rot_x.sin();
            float crx = transformTobeMapped.rot_x.cos();
            float sry = transformTobeMapped.rot_y.sin();
            float cry = transformTobeMapped.rot_y.cos();
            float srz = transformTobeMapped.rot_z.sin();
            float crz = transformTobeMapped.rot_z.cos();

            int laserCloudSelNum = laserCloudOri->points.size();
            if (laserCloudSelNum < 50) {
              continue;
            }

            Eigen::Matrix<float,Eigen::Dynamic,6> matA(laserCloudSelNum, 6);
            Eigen::Matrix<float,6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
            Eigen::Matrix<float,6, 6> matAtA;
            Eigen::VectorXf matB(laserCloudSelNum);
            Eigen::VectorXf matAtB;
            Eigen::VectorXf matX
                ;
            for (int i = 0; i < laserCloudSelNum; i++) {
              pointOri = laserCloudOri->points[i];
              coeff = coeffSel->points[i];

              float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                        + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                        + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

              float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                        + ((-cry*crz - srx*sry*srz)*pointOri.x 
                        + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

              float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                        + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                        + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

              matA(i, 0) = arx;
              matA(i, 1) = ary;
              matA(i, 2) = arz;
              matA(i, 3) = coeff.x;
              matA(i, 4) = coeff.y;
              matA(i, 5) = coeff.z;
              matB(i, 0) = -coeff.intensity;
            }
            matAt = matA.transpose();
            matAtA = matAt * matA;
            matAtB = matAt * matB;
            matX = matAtA.colPivHouseholderQr().solve(matAtB);

            if (iterCount == 0) {
              Eigen::Matrix<float,1, 6> matE;
              Eigen::Matrix<float,6, 6> matV;
              Eigen::Matrix<float,6, 6> matV2;

              Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
              matE = esolver.eigenvalues().real();
              matV = esolver.eigenvectors().real();

              matV2 = matV;

              isDegenerate = false;
              float eignThre[6] = {100, 100, 100, 100, 100, 100};
              for (int i = 5; i >= 0; i--) {
                if (matE(0, i) < eignThre[i]) {
                  for (int j = 0; j < 6; j++) {
                    matV2(i, j) = 0;
                  }
                  isDegenerate = true;
                } else {
                  break;
                }
              }
              matP = matV.inverse() * matV2;
            }

            if (isDegenerate) {
              Eigen::Matrix<float,6, 1> matX2(matX);
              matX = matP * matX2;
            }

            transformTobeMapped.rot_x += matX(0, 0);
            transformTobeMapped.rot_y += matX(1, 0);
            transformTobeMapped.rot_z += matX(2, 0);
            transformTobeMapped.pos.x() += matX(3, 0);
            transformTobeMapped.pos.y() += matX(4, 0);
            transformTobeMapped.pos.z() += matX(5, 0);

            float deltaR = sqrt(
                                pow(rad2deg(matX(0, 0)), 2) +
                                pow(rad2deg(matX(1, 0)), 2) +
                                pow(rad2deg(matX(2, 0)), 2));
            float deltaT = sqrt(
                                pow(matX(3, 0) * 100, 2) +
                                pow(matX(4, 0) * 100, 2) +
                                pow(matX(5, 0) * 100, 2));

            if (deltaR < 0.05 && deltaT < 0.05) {
              break;
            }
          }

          transformUpdate();
        }

        for (int i = 0; i < laserCloudCornerStackNum; i++) {
          pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

          int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

          if (pointSel.x + 25.0 < 0) cubeI--;
          if (pointSel.y + 25.0 < 0) cubeJ--;
          if (pointSel.z + 25.0 < 0) cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth && 
              cubeJ >= 0 && cubeJ < laserCloudHeight && 
              cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudCornerArray[cubeInd]->push_back(pointSel);
          }
        }

        for (int i = 0; i < laserCloudSurfStackNum; i++) {
          pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

          int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

          if (pointSel.x + 25.0 < 0) cubeI--;
          if (pointSel.y + 25.0 < 0) cubeJ--;
          if (pointSel.z + 25.0 < 0) cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth && 
              cubeJ >= 0 && cubeJ < laserCloudHeight && 
              cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudSurfArray[cubeInd]->push_back(pointSel);
          }
        }

        for (int i = 0; i < laserCloudValidNum; i++) {
          int ind = laserCloudValidInd[i];

          laserCloudCornerArray2[ind]->clear();
          downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
          downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

          laserCloudSurfArray2[ind]->clear();
          downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
          downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

          pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
          laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
          laserCloudCornerArray2[ind] = laserCloudTemp;

          laserCloudTemp = laserCloudSurfArray[ind];
          laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
          laserCloudSurfArray2[ind] = laserCloudTemp;
        }

        mapFrameCount++;
        if (mapFrameCount >= mapFrameNum) {
          mapFrameCount = 0;

          laserCloudSurround2->clear();
          for (int i = 0; i < laserCloudSurroundNum; i++) {
            int ind = laserCloudSurroundInd[i];
            *laserCloudSurround2 += *laserCloudCornerArray[ind];
            *laserCloudSurround2 += *laserCloudSurfArray[ind];
          }

          laserCloudSurround->clear();
          downSizeFilterCorner.setInputCloud(laserCloudSurround2);
          downSizeFilterCorner.filter(*laserCloudSurround);

          sensor_msgs::PointCloud2 laserCloudSurround3;
          pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
          laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
          laserCloudSurround3.header.frame_id = "/camera_init";
          pubLaserCloudSurround.publish(laserCloudSurround3);
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
          pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/camera_init";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  ( transformAftMapped.rot_z.value(),
                                   -transformAftMapped.rot_x.value(),
                                   -transformAftMapped.rot_y.value());

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped.pos.x();
        odomAftMapped.pose.pose.position.y = transformAftMapped.pos.y();
        odomAftMapped.pose.pose.position.z = transformAftMapped.pos.z();
        odomAftMapped.twist.twist.angular.x = transformBefMapped.rot_x.value();
        odomAftMapped.twist.twist.angular.y = transformBefMapped.rot_y.value();
        odomAftMapped.twist.twist.angular.z = transformBefMapped.rot_z.value();
        odomAftMapped.twist.twist.linear.x = transformBefMapped.pos.x();
        odomAftMapped.twist.twist.linear.y = transformBefMapped.pos.y();
        odomAftMapped.twist.twist.linear.z = transformBefMapped.pos.z();
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped.pos.x(),
                                             transformAftMapped.pos.y(),
                                             transformAftMapped.pos.z()));
        tfBroadcaster.sendTransform(aftMappedTrans);

      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

