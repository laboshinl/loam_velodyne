
#include <loam_velodyne/scanRegistrationLib.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/kdtree/kdtree_flann.h>

void ScanRegistration::run(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &originalLaserCloudIn, double scanTime,
    Outputs &outputs) {

  std::vector<int> indices;
  pcl::PointCloud<velodyne_pointcloud::VelodynePoint> laserCloudIn;
  pcl::removeNaNFromPointCloud(originalLaserCloudIn, laserCloudIn, indices);
  switchAxis(laserCloudIn);

  pcl::PointCloud<PointType> laserCloudOut;
  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  // laserCloudIn is ring split & concatenated back to laserCloudOut
  extractFeatures(laserCloudIn, scanTime, *outputs.laserCloudOut,
      *outputs.cornerPointsSharp, *outputs.cornerPointsLessSharp,
      *outputs.surfPointsFlat, *outputs.surfPointsLessFlat);
}

float ScanRegistration::computeStartHorizontalAngle(const velodyne_pointcloud::VelodynePoint &first_pt) {
  return -atan2(first_pt.x, first_pt.z);
}

float ScanRegistration::computeEndHorizontalAngle(const velodyne_pointcloud::VelodynePoint &last_pt, float start_angle) {
  float end_angle = -atan2(last_pt.x, last_pt.z) + 2 * M_PI;
  if (end_angle - start_angle > 3 * M_PI) {
    end_angle -= 2 * M_PI;
  } else if (end_angle - start_angle < M_PI) {
    end_angle += 2 * M_PI;
  }
  return end_angle;
}

// within the scan
float ScanRegistration::computeRelativeTime(const PointType &point, float start_angle, float end_angle, bool &half_passed) {
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

float ScanRegistration::computeCurvature(const pcl::PointCloud<PointType> &cloud, int idx) {
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

void ScanRegistration::switchAxis(pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloud) {
  for(pcl::PointCloud<velodyne_pointcloud::VelodynePoint>::iterator p = cloud.begin(); p < cloud.end(); p++) {
    float x = p->x;
    p->x = p->y;
    p->y = p->z;
    p->z = x;
  }
}

void ScanRegistration::extractFeatures(const pcl::PointCloud<velodyne_pointcloud::VelodynePoint> &cloudIn, double timeScanCur,
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
