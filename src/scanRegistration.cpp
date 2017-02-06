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
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <velodyne_pointcloud/point_types.h>

#include <loam_velodyne/LoamImu.h>

using std::sin;
using std::cos;
using std::atan2;
using namespace std;

const double scanPeriod = 0.1;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

const int N_SCANS = VELODYNE_MODEL;	// oring 16
const int MAX_POINTS = 200*1000;		// orig 40000

float cloudCurvature[MAX_POINTS];
int cloudSortInd[MAX_POINTS];
int cloudNeighborPicked[MAX_POINTS];
int cloudLabel[MAX_POINTS];

const int POINT_NEIGHBOURS = 5;

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubImuTrans;

LoamImuInput imu(scanPeriod);

float computeStartHorizontalAngle(const velodyne_pointcloud::PointXYZIR &first_pt) {
  return -atan2(first_pt.x, first_pt.z);
}

float computeEndHorizontalAngle(const velodyne_pointcloud::PointXYZIR &last_pt, float start_angle) {
  float end_angle = -atan2(last_pt.x, last_pt.z) + 2 * M_PI;
  if (end_angle - start_angle > 3 * M_PI) {
    end_angle -= 2 * M_PI;
  } else if (end_angle - start_angle < M_PI) {
    end_angle += 2 * M_PI;
  }
  return end_angle;
}

// within the scan
float computeRelativeTime(const PointType &point, float start_angle, float end_angle, bool &half_passed) {
  float ori = -atan2(point.x, point.z);
  if (!half_passed) {
    if (ori < start_angle - M_PI / 2) {
      ori += 2 * M_PI;
    } else if (ori > start_angle + M_PI * 3 / 2) {
      ori -= 2 * M_PI;
    }

    if (ori - start_angle > M_PI) {
      half_passed = true;
    }
  } else {
    ori += 2 * M_PI;

    if (ori < end_angle - M_PI * 3 / 2) {
      ori += 2 * M_PI;
    } else if (ori > end_angle + M_PI / 2) {
      ori -= 2 * M_PI;
    }
  }
  return (ori - start_angle) / (end_angle - start_angle);
}

float computeCurvature(const pcl::PointCloud<PointType> &cloud, int idx) {
  float diffX = 0;
  float diffY = 0;
  float diffZ = 0;
  for(int i = -POINT_NEIGHBOURS; i < POINT_NEIGHBOURS; i++) {
    if(i == 0) {
      diffX -= cloud[idx].x*2*POINT_NEIGHBOURS;
      diffY -= cloud[idx].y*2*POINT_NEIGHBOURS;
      diffZ -= cloud[idx].z*2*POINT_NEIGHBOURS;
    } else {
      diffX += cloud[idx + i].x;
      diffY += cloud[idx + i].y;
      diffZ += cloud[idx + i].z;
    }
  }
  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

void switchAxis(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud) {
  for(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::iterator p = cloud.begin(); p < cloud.end(); p++) {
    float x = p->x;
    p->x = p->y;
    p->y = p->z;
    p->z = x;
  }
}

void extractFeatures(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloudIn, double timeScanCur,
    pcl::PointCloud<PointType> &cloudOut,
    pcl::PointCloud<PointType> &cornerPointsSharp,
    pcl::PointCloud<PointType> &cornerPointsLessSharp,
    pcl::PointCloud<PointType> &surfPointsFlat,
    pcl::PointCloud<PointType> &surfPointsLessFlat) {
  const int cloudSize = cloudIn.size();
  float startOri = computeStartHorizontalAngle(cloudIn[0]);
  float endOri = computeEndHorizontalAngle(cloudIn.back(), startOri);

  bool halfPassed = false;
  PointType point;
  std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
  for (int i = 0; i < cloudSize; i++) {
    but_velodyne::copyXYZ(cloudIn[i], point);

    float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
    int scanID = cloudIn[i].ring;

    float relTime = computeRelativeTime(point, startOri, endOri, halfPassed);
    point.intensity = scanID + scanPeriod * relTime;

    if (imu.isAvailable()) {
      imu.improvePointPossition(point, timeScanCur, relTime, (i==0));
    }
    laserCloudScans[scanID].push_back(point);
  }

  for (int i = 0; i < N_SCANS; i++) {
    cloudOut += laserCloudScans[i];
  }

  // ring boundaries within the cumulative point cloud
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);
  int scanCount = -1;
  for (int i = POINT_NEIGHBOURS; i < cloudSize - POINT_NEIGHBOURS; i++) {
    cloudCurvature[i] = computeCurvature(cloudOut, i);
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    if (int(cloudOut[i].intensity) != scanCount) {
      scanCount = int(cloudOut[i].intensity);

      if (scanCount > 0 && scanCount < N_SCANS) {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }
  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;

  for (int i = 5; i < cloudSize - 6; i++) {
    float diffNext = pointsSqDistance(cloudOut[i], cloudOut[i + 1]);

    if (diffNext > 0.1) {

      float depthCur = pointNorm(cloudOut[i]);
      float depthNext = pointNorm(cloudOut[i+1]);

      if (depthCur > depthNext) {
        if (normalizedPointsDistance(cloudOut[i+1], 1.0, cloudOut[i], depthNext/depthCur) / depthNext < 0.1) {
          std::fill(&cloudNeighborPicked[i-POINT_NEIGHBOURS], &cloudNeighborPicked[i+1], 1);
        }
      } else {
        if (normalizedPointsDistance(cloudOut[i+1], depthCur/depthNext, cloudOut[i], 1.0) / depthCur < 0.1) {
          std::fill(&cloudNeighborPicked[i+1], &cloudNeighborPicked[i+POINT_NEIGHBOURS+2], 1);
        }
      }
    }

    float dist = pointSqNorm(cloudOut[i]);
    float diffPrev = pointsSqDistance(cloudOut[i], cloudOut[i - 1]);
    if (diffNext > 0.0002 * dist && diffPrev > 0.0002 * dist) {
      cloudNeighborPicked[i] = 1;
    }
  }

  for (int ring = 0; ring < N_SCANS; ring++) {
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {
      int sp = (scanStartInd[ring] * (6 - j)  + scanEndInd[ring] * j) / 6;
      int ep = (scanStartInd[ring] * (5 - j)  + scanEndInd[ring] * (j + 1)) / 6 - 1;

      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > 0.1) {
        
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(cloudOut[ind]);
            cornerPointsLessSharp.push_back(cloudOut[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp.push_back(cloudOut[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if (pointsSqDistance(cloudOut[ind + l], cloudOut[ind + l - 1]) > 0.05) {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (pointsSqDistance(cloudOut[ind + l], cloudOut[ind + l + 1]) > 0.05) {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;
          surfPointsFlat.push_back(cloudOut[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if (pointsSqDistance(cloudOut[ind + l], cloudOut[ind + l - 1]) > 0.05) {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (pointsSqDistance(cloudOut[ind + l], cloudOut[ind + l + 1]) > 0.05) {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(cloudOut[k]);
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

}

void processLasserCloud(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &originalLaserCloudIn, ros::Time stamp) {

  double timeScanCur = stamp.toSec();

  std::vector<int> indices;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> laserCloudIn;
  pcl::removeNaNFromPointCloud(originalLaserCloudIn, laserCloudIn, indices);
  switchAxis(laserCloudIn);

  pcl::PointCloud<PointType> laserCloudOut;
  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  // laserCloudIn is ring split & concatenated back to laserCloudOut
  extractFeatures(laserCloudIn, timeScanCur, laserCloudOut,
      cornerPointsSharp, cornerPointsLessSharp,
      surfPointsFlat, surfPointsLessFlat);

  publishCloud(laserCloudOut, pubLaserCloud, stamp, "/camera");
  publishCloud(cornerPointsSharp, pubCornerPointsSharp, stamp, "/camera");
  publishCloud(cornerPointsLessSharp, pubCornerPointsLessSharp, stamp, "/camera");
  publishCloud(surfPointsFlat, pubSurfPointsFlat, stamp, "/camera");
  publishCloud(surfPointsLessFlat, pubSurfPointsLessFlat, stamp, "/camera");

  sensor_msgs::PointCloud2 imuTransMsg;
  pcl::toROSMsg(imu.to4Points(), imuTransMsg);
  imuTransMsg.header.stamp = stamp;
  imuTransMsg.header.frame_id = "/camera";
  pubImuTrans.publish(imuTransMsg);
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
    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, &LoamImuInput::imuHandler, &imu);
    ros::spin();
  }

  return 0;
}
