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
#include <loam_velodyne/build_transform.h>

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
bool systemInited = false;

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

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>());

int laserCloudCornerLastNum;
int laserCloudSurfLastNum;

int pointSelCornerInd[40000];
float pointSearchCornerInd1[40000];
float pointSearchCornerInd2[40000];

int pointSelSurfInd[40000];
float pointSearchSurfInd1[40000];
float pointSearchSurfInd2[40000];
float pointSearchSurfInd3[40000];

float transformation[6] = {0};
float transformSum[6] = {0};

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;

void transformToStart(const PointType &pi, float *curr_transform, PointType &po)
{
  float phase = 10 * (pi.intensity - int(pi.intensity));
  po.getVector4fMap() = getTransformationTRzRxRy(curr_transform, -phase)*pi.getVector4fMap();
  po.intensity = pi.intensity;
}

void transformToEnd(pcl::PointCloud<PointType>::Ptr points, float *curr_transform) {

  Eigen::Affine3f fromStartToEnd =
      getTransformationRyRxRzT(0, 0, 0, imuPitchLast, imuYawLast, imuRollLast) *
      getTransformationTRzRxRy(-imuShiftFromStartX, -imuShiftFromStartY, -imuShiftFromStartZ, imuPitchStart, imuYawStart, imuRollStart) *
      getTransformationRyRxRzT(curr_transform);

  for(pcl::PointCloud<PointType>::iterator p = points->begin(); p < points->end(); p++) {
    transformToStart(*p, curr_transform, *p);
    p->getVector4fMap() = fromStartToEnd*p->getVector4fMap();
  }
}

void pluginIMURotation(float bcx, float bcy, float bcz,
                       float blx, float bly, float blz,
                       float alx, float aly, float alz,
                       float &acx, float &acy, float &acz) {
  Eigen::Affine3f current = pcl::getTransformation(0, 0, 0, bcy, bcx, bcz);
  Eigen::Affine3f before = pcl::getTransformation(0, 0, 0, bly, blx, blz);
  Eigen::Affine3f after = pcl::getTransformation(0, 0, 0, aly, alx, alz);

  Eigen::Affine3f output = after * before.inverse() * current;
  pcl::getEulerAngles(output, acy, acx, acz);
}

void accumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                        float &ox, float &oy, float &oz) {
  Eigen::Affine3f current = pcl::getTransformation(0, 0, 0, cy, cx, cz);
  Eigen::Affine3f last = pcl::getTransformation(0, 0, 0, ly, lx, lz);

  pcl::getEulerAngles(last*current, oy, ox, oz);
}

void accumulateTransformation(const float *t_increment, float *t_sum) {
  float rx, ry, rz;
  accumulateRotation(t_sum[0], t_sum[1], t_sum[2],
                     -t_increment[0], -t_increment[1] * 1.05, -t_increment[2],
                     rx, ry, rz);

  Eigen::Vector4f pos;
  pos << t_increment[5] * 1.05 - imuShiftFromStartZ,
      t_increment[3] - imuShiftFromStartX,
      t_increment[4] - imuShiftFromStartY, 1;
  pos = pcl::getTransformation(0, 0, 0, rz, rx, ry).matrix() * pos;
  t_sum[3] -= pos(1);
  t_sum[4] -= pos(2);
  t_sum[5] -= pos(0);

  pluginIMURotation(rx, ry, rz,
                    imuPitchStart, imuYawStart, imuRollStart,
                    imuPitchLast, imuYawLast, imuRollLast,
                    t_sum[0], t_sum[1], t_sum[2]);
}

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharp2)
{
  loadCloudFromMsg(cornerPointsSharp2, cornerPointsSharp, timeCornerPointsSharp);
  newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharp2)
{
  loadCloudFromMsg(cornerPointsLessSharp2, cornerPointsLessSharp, timeCornerPointsLessSharp);
  newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlat2)
{
  loadCloudFromMsg(surfPointsFlat2, surfPointsFlat, timeSurfPointsFlat);
  newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlat2)
{
  loadCloudFromMsg(surfPointsLessFlat2, surfPointsLessFlat, timeSurfPointsLessFlat);
  newSurfPointsLessFlat = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
  loadCloudFromMsg(laserCloudFullRes2, laserCloudFullRes, timeLaserCloudFullRes);
  newLaserCloudFullRes = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2)
{
  timeImuTrans = imuTrans2->header.stamp.toSec();

  imuTrans->clear();
  pcl::fromROSMsg(*imuTrans2, *imuTrans);

  imuPitchStart = imuTrans->points[0].x;
  imuYawStart = imuTrans->points[0].y;
  imuRollStart = imuTrans->points[0].z;

  imuPitchLast = imuTrans->points[1].x;
  imuYawLast = imuTrans->points[1].y;
  imuRollLast = imuTrans->points[1].z;

  imuShiftFromStartX = imuTrans->points[2].x;
  imuShiftFromStartY = imuTrans->points[2].y;
  imuShiftFromStartZ = imuTrans->points[2].z;

  imuVeloFromStartX = imuTrans->points[3].x;
  imuVeloFromStartY = imuTrans->points[3].y;
  imuVeloFromStartZ = imuTrans->points[3].z;

  newImuTrans = true;
}

/**
 * Line is given by points AB.
 * The result is the distance and the direction to closest point from the third point X.
 */
float getLinePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
    const Eigen::Vector3f &X, Eigen::Vector3f &unit_direction) {
  Eigen::Vector3f BXcrossAX = (X-B).cross(X-A);
  float BXcrossAXnorm = BXcrossAX.norm();
  float lengthAB = (A-B).norm();
  unit_direction = -BXcrossAX.cross(B-A) / (BXcrossAXnorm * lengthAB);
  return BXcrossAXnorm / lengthAB;
}

bool getCornerFeatureCoefficients(const PointType &A, const PointType &B,
    const PointType &X, int iterration, PointType &coeff) {
  Eigen::Vector3f direction;
  float distance = getLinePointDistance(A.getVector3fMap(), B.getVector3fMap(), X.getVector3fMap(), direction);

  float weight = 1.0;
  if (iterration >= 5) {
    weight = 1 - 1.8 * fabs(distance);
  }

  coeff.getVector3fMap() = direction * weight;
  coeff.intensity = distance * weight;

  return (weight > 0.1 && distance != 0);
}

float getSurfacePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C,
    const Eigen::Vector3f &X, Eigen::Vector3f &surfNormal) {
  surfNormal = (B-A).cross(C-A);
  surfNormal.normalize();

  float normalDotA = -surfNormal.dot(A);
  float distance = surfNormal.dot(X) + normalDotA;
  return distance;
}

bool getSurfaceFeatureCoefficients(const PointType &A, const PointType &B, const PointType &C,
    const PointType &X, int iterration, PointType &coefficients) {
  Eigen::Vector3f surfNormal;
  float distance = getSurfacePointDistance(A.getVector3fMap(), B.getVector3fMap(), C.getVector3fMap(),
      X.getVector3fMap(), surfNormal);

  float weight = 1;
  if (iterration >= 5) {
    weight = 1 - 1.8 * fabs(distance) / sqrt(X.getVector3fMap().norm());
  }
  coefficients.getVector3fMap() = weight * surfNormal;
  coefficients.intensity = weight * distance;

  return (weight > 0.1 && distance != 0);
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

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  int frameCount = skipFrameNum;
  ros::Rate rate(100);
  bool status = ros::ok();
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
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        publishCloud(*laserCloudCornerLast, pubLaserCloudCornerLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
        publishCloud(*laserCloudSurfLast, pubLaserCloudSurfLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");

        transformSum[0] += imuPitchStart;
        transformSum[2] += imuRollStart;

        systemInited = true;
        continue;
      }

      transformation[3] -= imuVeloFromStartX * scanPeriod;
      transformation[4] -= imuVeloFromStartY * scanPeriod;
      transformation[5] -= imuVeloFromStartZ * scanPeriod;

      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cornerPointsSharp,*cornerPointsSharp, indices);
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum = surfPointsFlat->points.size();
        for (int iterCount = 0; iterCount < 25; iterCount++) {
          laserCloudOri->clear();
          coeffSel->clear();

          for (int i = 0; i < cornerPointsSharpNum; i++) {
            PointType pointSel;
            transformToStart(cornerPointsSharp->points[i], transformation, pointSel);

            if (iterCount % 5 == 0) {
              std::vector<int> indices;
              pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices);
              kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
              int closestPointInd = -1, minPointInd2 = -1;
              if (pointSearchSqDis[0] < 25) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = 25;
                for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                  if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                    break;
                  }

                  pointSqDis = pointsSqDistance(laserCloudCornerLast->points[j], pointSel);

                  if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis = pointsSqDistance(laserCloudCornerLast->points[j], pointSel);

                  if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
              }

              pointSearchCornerInd1[i] = closestPointInd;
              pointSearchCornerInd2[i] = minPointInd2;
            }

            if (pointSearchCornerInd2[i] >= 0) {
              const PointType &A = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
              const PointType &B = laserCloudCornerLast->points[pointSearchCornerInd2[i]];
              PointType coefficients;
              if(getCornerFeatureCoefficients(A, B, pointSel, iterCount, coefficients)) {
                laserCloudOri->push_back(cornerPointsSharp->points[i]);
                coeffSel->push_back(coefficients);
              }
            }
          }

          for (int i = 0; i < surfPointsFlatNum; i++) {
            PointType pointSel;
            transformToStart(surfPointsFlat->points[i], transformation, pointSel);

            if (iterCount % 5 == 0) {
              kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
              int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
              if (pointSearchSqDis[0] < 25) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

                float minPointSqDis2 = 25, minPointSqDis3 = 25;
                for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                  if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                    break;
                  }

                  float pointSqDis = pointsSqDistance(laserCloudSurfLast->points[j], pointSel);

                  if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                     if (pointSqDis < minPointSqDis2) {
                       minPointSqDis2 = pointSqDis;
                       minPointInd2 = j;
                     }
                  } else {
                     if (pointSqDis < minPointSqDis3) {
                       minPointSqDis3 = pointSqDis;
                       minPointInd3 = j;
                     }
                  }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                    break;
                  }

                  float pointSqDis = pointsSqDistance(laserCloudSurfLast->points[j], pointSel);

                  if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  } else {
                    if (pointSqDis < minPointSqDis3) {
                      minPointSqDis3 = pointSqDis;
                      minPointInd3 = j;
                    }
                  }
                }
              }

              pointSearchSurfInd1[i] = closestPointInd;
              pointSearchSurfInd2[i] = minPointInd2;
              pointSearchSurfInd3[i] = minPointInd3;
            }

            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
              const PointType &A = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
              const PointType &B = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
              const PointType &C = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

              PointType coefficients;
              if(getSurfaceFeatureCoefficients(A, B, C, pointSel, iterCount, coefficients)) {
                laserCloudOri->push_back(surfPointsFlat->points[i]);
                coeffSel->push_back(coefficients);
              }
            }
          }

          int pointSelNum = laserCloudOri->points.size();
          if (pointSelNum < 10) {
            continue;
          }

          cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
          for (int i = 0; i < pointSelNum; i++) {
            PointType &pointOri = laserCloudOri->points[i];
            PointType &coeff = coeffSel->points[i];

            float srx = sin(transformation[0]);
            float crx = cos(transformation[0]);
            float sry = sin(transformation[1]);
            float cry = cos(transformation[1]);
            float srz = sin(transformation[2]);
            float crz = cos(transformation[2]);
            float tx = transformation[3];
            float ty = transformation[4];
            float tz = transformation[5];

            float arx = (-crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y + srx*sry*pointOri.z
                      + tx*crx*sry*srz - ty*crx*crz*sry - tz*srx*sry) * coeff.x
                      + (srx*srz*pointOri.x - crz*srx*pointOri.y + crx*pointOri.z
                      + ty*crz*srx - tz*crx - tx*srx*srz) * coeff.y
                      + (crx*cry*srz*pointOri.x - crx*cry*crz*pointOri.y - cry*srx*pointOri.z
                      + tz*cry*srx + ty*crx*cry*crz - tx*crx*cry*srz) * coeff.z;

            float ary = ((-crz*sry - cry*srx*srz)*pointOri.x
                      + (cry*crz*srx - sry*srz)*pointOri.y - crx*cry*pointOri.z
                      + tx*(crz*sry + cry*srx*srz) + ty*(sry*srz - cry*crz*srx)
                      + tz*crx*cry) * coeff.x
                      + ((cry*crz - srx*sry*srz)*pointOri.x
                      + (cry*srz + crz*srx*sry)*pointOri.y - crx*sry*pointOri.z
                      + tz*crx*sry - ty*(cry*srz + crz*srx*sry)
                      - tx*(cry*crz - srx*sry*srz)) * coeff.z;

            float arz = ((-cry*srz - crz*srx*sry)*pointOri.x + (cry*crz - srx*sry*srz)*pointOri.y
                      + tx*(cry*srz + crz*srx*sry) - ty*(cry*crz - srx*sry*srz)) * coeff.x
                      + (-crx*crz*pointOri.x - crx*srz*pointOri.y
                      + ty*crx*srz + tx*crx*crz) * coeff.y
                      + ((cry*crz*srx - sry*srz)*pointOri.x + (crz*sry + cry*srx*srz)*pointOri.y
                      + tx*(sry*srz - cry*crz*srx) - ty*(crz*sry + cry*srx*srz)) * coeff.z;

            float atx = -(cry*crz - srx*sry*srz) * coeff.x + crx*srz * coeff.y
                      - (crz*sry + cry*srx*srz) * coeff.z;
  
            float aty = -(cry*srz + crz*srx*sry) * coeff.x - crx*crz * coeff.y
                      - (sry*srz - cry*crz*srx) * coeff.z;
  
            float atz = crx*sry * coeff.x - srx * coeff.y - crx*cry * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;
            matB.at<float>(i, 0) = -0.05 * d2;
          }
          cv::transpose(matA, matAt);
          matAtA = matAt * matA;
          matAtB = matAt * matB;
          cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

          if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) {
              if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                  matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
              } else {
                break;
              }
            }
            matP = matV.inv() * matV2;
          }

          if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
          }

          for(int i = 0; i < 6; i++){
            transformation[i] += matX.at<float>(i, 0);
            if(isnan(transformation[i])) {
              transformation[i] = 0;
            }
          }

          float deltaR = norm(matX.rowRange(0, 3));
          float deltaT = norm(matX.rowRange(3, 6)) * 100;

          if (deltaR < DEG2RAD(0.1) && deltaT < 0.1) {
            break;
          }
        }
      }

      accumulateTransformation(transformation, transformSum);

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

      laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      laserOdometry.pose.pose.orientation.x = -geoQuat.y;
      laserOdometry.pose.pose.orientation.y = -geoQuat.z;
      laserOdometry.pose.pose.orientation.z = geoQuat.x;
      laserOdometry.pose.pose.orientation.w = geoQuat.w;
      laserOdometry.pose.pose.position.x = transformSum[3];
      laserOdometry.pose.pose.position.y = transformSum[4];
      laserOdometry.pose.pose.position.z = transformSum[5];
      pubLaserOdometry.publish(laserOdometry);

      laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
      laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
      tfBroadcaster.sendTransform(laserOdometryTrans);

      transformToEnd(cornerPointsLessSharp, transformation);
      transformToEnd(surfPointsLessFlat, transformation);

      frameCount++;
      if (frameCount >= skipFrameNum + 1) {
        transformToEnd(laserCloudFullRes, transformation);
      }

      pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
      cornerPointsLessSharp = laserCloudCornerLast;
      laserCloudCornerLast = laserCloudTemp;

      laserCloudTemp = surfPointsLessFlat;
      surfPointsLessFlat = laserCloudSurfLast;
      laserCloudSurfLast = laserCloudTemp;

      laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      laserCloudSurfLastNum = laserCloudSurfLast->points.size();
      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
      }

      if (frameCount >= skipFrameNum + 1) {
        frameCount = 0;
        publishCloud(*laserCloudCornerLast, pubLaserCloudCornerLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
        publishCloud(*laserCloudSurfLast, pubLaserCloudSurfLast, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
        publishCloud(*laserCloudFullRes, pubLaserCloudFullRes, ros::Time().fromSec(timeSurfPointsLessFlat), "/camera");
      }

      ROS_DEBUG_STREAM("[laserOdometry] took " << stopWatch.elapsed());
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
