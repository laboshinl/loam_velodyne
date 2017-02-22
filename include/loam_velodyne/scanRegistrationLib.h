#ifndef __LOAM_SCAN_REGISTRATION_H__
#define __LOAM_SCAN_REGISTRATION_H__

#include <loam_velodyne/LoamImu.h>

#include <vector>
#include <cmath>
#include <pcl/point_types.h>

class ScanRegistration {
public:
  class Outputs {
  public:
    pcl::PointCloud<PointType>::Ptr laserCloudOut;
    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

    Outputs() :
      laserCloudOut(new pcl::PointCloud<PointType>),
      cornerPointsSharp(new pcl::PointCloud<PointType>),
      cornerPointsLessSharp(new pcl::PointCloud<PointType>),
      surfPointsFlat(new pcl::PointCloud<PointType>),
      surfPointsLessFlat(new pcl::PointCloud<PointType>) {
    }
  };

private:
  double scanPeriod;

  static const int N_SCANS = VELODYNE_MODEL; // oring 16
  static const int MAX_POINTS = 200*1000;        // orig 40000

  float cloudCurvature[MAX_POINTS];
  int cloudSortInd[MAX_POINTS];
  int cloudNeighborPicked[MAX_POINTS];
  int cloudLabel[MAX_POINTS];

  static const int POINT_NEIGHBOURS = 5;

  LoamImuInput &imu;

public:
  ScanRegistration(LoamImuInput &imu_, double scanPeriod_) :
    imu(imu_), scanPeriod(scanPeriod_) {
  }

  void run(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &originalLaserCloudIn, double scanTime,
      Outputs &outputs);

protected:
  float computeStartHorizontalAngle(const velodyne_pointcloud::PointXYZIR &first_pt);

  float computeEndHorizontalAngle(const velodyne_pointcloud::PointXYZIR &last_pt, float start_angle);

  // within the scan
  float computeRelativeTime(const PointType &point, float start_angle, float end_angle, bool &half_passed);

  float computeCurvature(const pcl::PointCloud<PointType> &cloud, int idx);

  void switchAxis(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud);

  void extractFeatures(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloudIn, double timeScanCur,
      pcl::PointCloud<PointType> &cloudOut,
      pcl::PointCloud<PointType> &cornerPointsSharp,
      pcl::PointCloud<PointType> &cornerPointsLessSharp,
      pcl::PointCloud<PointType> &surfPointsFlat,
      pcl::PointCloud<PointType> &surfPointsLessFlat);
};

#endif
