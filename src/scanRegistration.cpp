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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using std::sin;
using std::cos;
using std::atan2;

#ifndef VELODYNE_HDL64E
const double scanPeriod = 0.1; // time duration per scan
#else
const double scanPeriod = 0.1; // TODO
#endif

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

#ifndef VELODYNE_HDL64E
const int N_SCANS = 16; /////
#else
const int N_SCANS = 64;
#endif

#ifndef VELODYNE_HDL64E
const int MAX_POINTS = 40000;
#else
const int MAX_POINTS = 160000;
#endif

float cloudCurvature[MAX_POINTS];
int cloudSortInd[MAX_POINTS];
int cloudNeighborPicked[MAX_POINTS];
int cloudLabel[MAX_POINTS];

int imuPointerFront = 0;
int imuPointerLast = -1;

#ifndef VELODYNE_HDL64E
const int imuQueLength = 200;
#else
const int imuQueLength = 2000;
#endif

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0,
      imuShiftFromStartZCur = 0; // updated by ShiftToStartIMU()
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0,
      imuVeloFromStartZCur = 0; // updated by VeloToStartIMU()

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};
// updated by AccumulateIMUShift()
float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};
// updated by AccumulateIMUShift()
float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubImuTrans;

// imu shift from start vector (imuShiftFromStart*Cur) converted into start imu
// coordinates?
void ShiftToStartIMU(float pointTime) {
  imuShiftFromStartXCur =
      imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
  imuShiftFromStartYCur =
      imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
  imuShiftFromStartZCur =
      imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

  float x1 = cos(imuYawStart) * imuShiftFromStartXCur -
             sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur +
             cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}
// imu velocity from start vector (imuVeloFromStart*Cur) converted into start
// imu coordinates?
void VeloToStartIMU() {
  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  float x1 = cos(imuYawStart) * imuVeloFromStartXCur -
             sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur +
             cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}
// points converted into start imu coordinates?
void TransformToStartIMU(PointType *p) {
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x =
      cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y =
      -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}
// compute last shift to imuShift*[imuPointerLast] and velo to
// imuVelo*[imuPointerLast] using previous shift/velo/acc
void AccumulateIMUShift() {
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < scanPeriod) {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] +
                                imuVeloX[imuPointerBack] * timeDiff +
                                accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] +
                                imuVeloY[imuPointerBack] * timeDiff +
                                accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] +
                                imuVeloZ[imuPointerBack] * timeDiff +
                                accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}

auto last_time = std::chrono::system_clock::now();

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    }
    return;
  }

  std::vector<int> scanStartInd(
      N_SCANS, 0); // scanStartInd[scanId] is the first point id of scanId
  std::vector<int> scanEndInd(
      N_SCANS, 0); // scanEndInd[scanId] is the last point id of scanId

  double timeScanCur =
      laserCloudMsg->header.stamp.toSec(); // time point of current scan
  pcl::PointCloud<pcl::PointXYZ>
      laserCloudIn; // input cloud, NaN points removed
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

  //ROS_INFO("cloud recieved");
  if (false) {
    // write clound to file
    static bool written = false;
    if (!written) {
      std::ofstream ofs("/home/i-yanghao/tmp/normalized_cloud.xyz");
      if (ofs) {
        for (int i = 0; i < laserCloudIn.points.size(); i++) {
          auto & p = laserCloudIn.points[i];
          float len = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
          ofs << p.x / len << " " << p.y / len << " " << p.z / len << std::endl;
        }
        ROS_INFO("cloud written");
        written = true;
      }
    }
  }

  int cloudSize = laserCloudIn.points.size(); // number of cloud points
  float startOri =
      -atan2(laserCloudIn.points[0].y,
             laserCloudIn.points[0]
                 .x); // ori of first point in cloud on origin x-y plane
  float endOri =
      -atan2(laserCloudIn.points[cloudSize - 1]
                 .y, // ori of last point in clound on origin x-y plane
             laserCloudIn.points[cloudSize - 1].x) +
      2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

  // float minAngle = 180, maxAngle = -180;
  // PointType minP, maxP;
  // minP.x = minP.y = minP.z = 1e8;
  // maxP.x = maxP.y = maxP.z = -1e8;

  /// use imu data to register original scanned points into lidar coodinates in
  /// different scan lines
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].y;
    point.y = laserCloudIn.points[i].z;
    point.z = laserCloudIn.points[i].x;

    // minP.x = std::min(minP.x, point.x);
    // minP.y = std::min(minP.y, point.y);
    // minP.z = std::min(minP.z, point.z);
    // maxP.x = std::max(maxP.x, point.x);
    // maxP.y = std::max(maxP.y, point.y);
    // maxP.z = std::max(maxP.z, point.z);

    float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) *
                  180 / M_PI; // angle of origin z from origin x-y plane
    int scanID;
    // if(!std::isnan(angle)) {
    //   minAngle = std::min(angle, minAngle);
    //   maxAngle = std::max(angle, maxAngle);
    // }
    // ROS_INFO("[%f]", angle);

    // compute scanID
#ifndef VELODYNE_HDL64E
    int roundedAngle = int(angle + (angle < 0.0 ? -0.5 : +0.5));
    if (roundedAngle > 0) {
      scanID = roundedAngle;
    } else {
      scanID = roundedAngle + (N_SCANS - 1);
    }
#else
    const float angleLowerBoundDeg = -24.8f;
    const float angleUpperBoundDeg = 2.0f;
    const float angleSpan = angleUpperBoundDeg - angleLowerBoundDeg;
    const float angleStep = angleSpan / (N_SCANS - 1);
    float angleID = (angle - angleLowerBoundDeg) / angleStep;
    
    scanID = int(angleID + 0.5f);
#endif

    if (scanID > (N_SCANS - 1) || scanID < 0) { // drop the points with invalid scanIDs
      count--;
      continue;
    }

    const int debug_errorPointIDStart = 121513;
    // if (i >= debug_errorPointIDStart) {
    //   ROS_INFO("point %i's scanID = %i", i, scanID);
    // }    

    float ori = -atan2(point.x, point.z);

    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + scanPeriod * relTime;

    // if (i >= debug_errorPointIDStart) {
    //   ROS_INFO("halfPassed = %i, ori = %f, point intensity = %f", halfPassed,
    //            ori, point.intensity);
    // }

    if (imuPointerLast >= 0) {
      float pointTime = relTime * scanPeriod;
      while (imuPointerFront != imuPointerLast) {
        if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
          break;
        }
        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }

      if (timeScanCur + pointTime >
          imuTime[imuPointerFront]) { /// use the newest imu data if no newer
                                      /// imu
        imuRollCur = imuRoll[imuPointerFront];
        imuPitchCur = imuPitch[imuPointerFront];
        imuYawCur = imuYaw[imuPointerFront];

        imuVeloXCur = imuVeloX[imuPointerFront];
        imuVeloYCur = imuVeloY[imuPointerFront];
        imuVeloZCur = imuVeloZ[imuPointerFront];

        imuShiftXCur = imuShiftX[imuPointerFront];
        imuShiftYCur = imuShiftY[imuPointerFront];
        imuShiftZCur = imuShiftZ[imuPointerFront];
      } else { /// interpolate in all existing imu data if there are newer imu
               /// data
        int imuPointerBack =
            (imuPointerFront + imuQueLength - 1) % imuQueLength;
        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) /
                           (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) /
                          (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

        imuRollCur = imuRoll[imuPointerFront] * ratioFront +
                     imuRoll[imuPointerBack] * ratioBack;
        imuPitchCur = imuPitch[imuPointerFront] * ratioFront +
                      imuPitch[imuPointerBack] * ratioBack;
        if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront +
                      (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
        } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront +
                      (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
        } else {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront +
                      imuYaw[imuPointerBack] * ratioBack;
        }

        imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront +
                      imuVeloX[imuPointerBack] * ratioBack;
        imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront +
                      imuVeloY[imuPointerBack] * ratioBack;
        imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront +
                      imuVeloZ[imuPointerBack] * ratioBack;

        imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront +
                       imuShiftX[imuPointerBack] * ratioBack;
        imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront +
                       imuShiftY[imuPointerBack] * ratioBack;
        imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront +
                       imuShiftZ[imuPointerBack] * ratioBack;
      }
      if (i == 0) {
        imuRollStart = imuRollCur;
        imuPitchStart = imuPitchCur;
        imuYawStart = imuYawCur;

        imuVeloXStart = imuVeloXCur;
        imuVeloYStart = imuVeloYCur;
        imuVeloZStart = imuVeloZCur;

        imuShiftXStart = imuShiftXCur;
        imuShiftYStart = imuShiftYCur;
        imuShiftZStart = imuShiftZCur;
      } else {
        ShiftToStartIMU(pointTime);
        VeloToStartIMU();
        TransformToStartIMU(&point);
      }
    }

    laserCloudScans[scanID].push_back(point);
  }

  //ROS_INFO("all points are grouped");

  // ROS_INFO("\n");
  // ROS_INFO("minAngle = %f, maxAngle = %f\n", minAngle, maxAngle);
  // output minAngle = -15, maxAngle = 15
  // ROS_INFO("bounding box = [%f,%f,%f; %f,%f,%f]\n", minP.x, minP.y, minP.z,
  // maxP.x, maxP.y, maxP.z);
  // output generally: [-20(+-10), -5(+-1), -100(+-20); +70(+-10), +25(+-1),
  // +80(+-10)]

  cloudSize = count;

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < N_SCANS; i++) {
    *laserCloud += laserCloudScans[i];
  }
  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++) {
    //ROS_INFO("i = %i, cloundSize = %i", i, cloudSize);
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x +
                  laserCloud->points[i - 3].x + laserCloud->points[i - 2].x +
                  laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                  laserCloud->points[i + 1].x + laserCloud->points[i + 2].x +
                  laserCloud->points[i + 3].x + laserCloud->points[i + 4].x +
                  laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y +
                  laserCloud->points[i - 3].y + laserCloud->points[i - 2].y +
                  laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                  laserCloud->points[i + 1].y + laserCloud->points[i + 2].y +
                  laserCloud->points[i + 3].y + laserCloud->points[i + 4].y +
                  laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z +
                  laserCloud->points[i - 3].z + laserCloud->points[i - 2].z +
                  laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                  laserCloud->points[i + 1].z + laserCloud->points[i + 2].z +
                  laserCloud->points[i + 3].z + laserCloud->points[i + 4].z +
                  laserCloud->points[i + 5].z;
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    if (int(laserCloud->points[i].intensity) != scanCount) {
      scanCount = int(laserCloud->points[i].intensity);

      if (scanCount > 0 && scanCount < N_SCANS) {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }
  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;

  //ROS_INFO("cloudCurvature scanStartInd scanEndInd computed");

  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {
      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                          laserCloud->points[i].y * laserCloud->points[i].y +
                          laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 =
          sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
               laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
               laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x -
                laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y -
                laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z -
                laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 <
            0.1) { // is connected?
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 -
                laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 -
                laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 -
                laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 <
            0.1) { // is connected?
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x +
                laserCloud->points[i].y * laserCloud->points[i].y +
                laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  //ROS_INFO("cloudNeighborPicked initialized");

  pcl::PointCloud<PointType> cornerPointsSharp;     // the outputs
  pcl::PointCloud<PointType> cornerPointsLessSharp; // the outputs
  pcl::PointCloud<PointType> surfPointsFlat;        // the outputs
  pcl::PointCloud<PointType> surfPointsLessFlat;    // the outputs

  for (int i = 0; i < N_SCANS; i++) {
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(
        new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {

      int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;
      int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

      for (int k = sp + 1; k <= ep; k++) { // sort by curvature within [sp,
                                           // ep]?, curvature descending order
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] <
              cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(laserCloud->points[ind]);
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;
          surfPointsFlat.push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;
  }

  //ROS_INFO("feature points collected");

  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/camera";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "/camera";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsLessSharpMsg.header.frame_id = "/camera";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "/camera";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/camera";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

  pcl::PointCloud<pcl::PointXYZ> imuTrans(4, 1);
  imuTrans.points[0].x = imuPitchStart;
  imuTrans.points[0].y = imuYawStart;
  imuTrans.points[0].z = imuRollStart;

  imuTrans.points[1].x = imuPitchCur;
  imuTrans.points[1].y = imuYawCur;
  imuTrans.points[1].z = imuRollCur;

  imuTrans.points[2].x = imuShiftFromStartXCur;
  imuTrans.points[2].y = imuShiftFromStartYCur;
  imuTrans.points[2].z = imuShiftFromStartZCur;

  imuTrans.points[3].x = imuVeloFromStartXCur;
  imuTrans.points[3].y = imuVeloFromStartYCur;
  imuTrans.points[3].z = imuVeloFromStartZCur;

  sensor_msgs::PointCloud2 imuTransMsg;
  pcl::toROSMsg(imuTrans, imuTransMsg);
  imuTransMsg.header.stamp = laserCloudMsg->header.stamp;
  imuTransMsg.header.frame_id = "/camera";
  pubImuTrans.publish(imuTransMsg);

  // #define PRINT(name) ROS_INFO("in scanRegistration "#name" = %f", name)
  //  PRINT(imuShiftFromStartXCur);
  //  PRINT(imuShiftFromStartYCur);
  //  PRINT(imuShiftFromStartZCur);
  //  PRINT(imuVeloFromStartXCur);
  //  PRINT(imuVeloFromStartYCur);
  //  PRINT(imuVeloFromStartZCur);
  // #undef PRINT
}

void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn) {
  //ROS_INFO("imu recieved!\n");
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  //#define PRINT(name) ROS_INFO(#name" = %f\n", name)
  //  PRINT(accX);
  //  PRINT(accY);
  //  PRINT(accZ);
  //#undef PRINT

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  AccumulateIMUShift();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 2, laserCloudHandler);

  ros::Subscriber subImu =
      nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imuHandler);

  pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);

  pubCornerPointsSharp =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);

  pubCornerPointsLessSharp =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);

  pubSurfPointsFlat =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);

  pubSurfPointsLessFlat =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);

  pubImuTrans = nh.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  ros::spin();

  return 0;
}
