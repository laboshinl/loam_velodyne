#ifndef __LOAM_LASER_ODOMETRY_H__
#define __LOAM_LASER_ODOMETRY_H__

#include <loam_velodyne/common.h>
#include <loam_velodyne/build_transform.h>
#include <loam_velodyne/scanRegistrationLib.h>

#include <opencv/cv.h>
#include <pcl/kdtree/kdtree_flann.h>

class LaserOdometry {
private:
  const float scanPeriod;

  float imuRollStart, imuPitchStart, imuYawStart;
  float imuRollLast, imuPitchLast, imuYawLast;
  float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
  float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

  pcl::KdTreeFLANN<PointType> kdtreeCornerLast;
  pcl::KdTreeFLANN<PointType> kdtreeSurfLast;

  bool isDegenerate;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  static const int MAX_POINTS = 40000;
  float pointSearchCornerInd1[MAX_POINTS];
  float pointSearchCornerInd2[MAX_POINTS];

  float pointSearchSurfInd1[MAX_POINTS];
  float pointSearchSurfInd2[MAX_POINTS];
  float pointSearchSurfInd3[MAX_POINTS];

  static const int DOF = 6;
  float transformation[DOF];
  float transformSum[DOF];

public:
  class Inputs : public ScanRegistration::Outputs {};

  class Outputs {
  public:
    pcl::PointCloud<PointType>::Ptr corners, surfels, cloud;
    std::vector<float> t;
    Outputs() :
      corners(new pcl::PointCloud<PointType>),
      surfels(new pcl::PointCloud<PointType>),
      cloud(new pcl::PointCloud<PointType>),
      t(6, 0) {
    }
  };

  LaserOdometry(float scanPeriod_) :
    scanPeriod(scanPeriod_),
    imuRollStart(0.0), imuPitchStart(0.0), imuYawStart(0.0),
    imuRollLast(0.0), imuPitchLast(0.0), imuYawLast(0.0),
    imuShiftFromStartX(0.0), imuShiftFromStartY(0.0), imuShiftFromStartZ(0.0),
    imuVeloFromStartX(0.0), imuVeloFromStartY(0.0), imuVeloFromStartZ(0.0),
    isDegenerate(false),
    laserCloudCornerLastNum(0),
    laserCloudSurfLastNum(0),
    laserCloudCornerLast(new pcl::PointCloud<PointType>),
    laserCloudSurfLast(new pcl::PointCloud<PointType>) {

    std::fill(transformation, &(transformation[DOF]), 0.0);
    std::fill(transformSum, &(transformSum[DOF]), 0.0);

    std::fill(pointSearchCornerInd1, &(pointSearchCornerInd1[MAX_POINTS]), 0.0);
    std::fill(pointSearchCornerInd2, &(pointSearchCornerInd2[MAX_POINTS]), 0.0);
    std::fill(pointSearchSurfInd1, &(pointSearchSurfInd1[MAX_POINTS]), 0.0);
    std::fill(pointSearchSurfInd2, &(pointSearchSurfInd2[MAX_POINTS]), 0.0);
    std::fill(pointSearchSurfInd3, &(pointSearchSurfInd3[MAX_POINTS]), 0.0);
  }

  void run(LaserOdometry::Inputs &inputs, LaserOdometry::Outputs &outputs);

  void updateImu(const pcl::PointCloud<pcl::PointXYZ> &imuTrans);

protected:
  void transformToStart(const PointType &pi, float *curr_transform, PointType &po);

  void transformToEnd(pcl::PointCloud<PointType>::Ptr points, float *curr_transform);

  void pluginIMURotation(float bcx, float bcy, float bcz,
                         float blx, float bly, float blz,
                         float alx, float aly, float alz,
                         float &acx, float &acy, float &acz);

  void accumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                          float &ox, float &oy, float &oz);

  void accumulateTransformation(const float *t_increment, float *t_sum);

  /**
   * Line is given by points AB.
   * The result is the distance and the direction to closest point from the third point X.
   */
  float getLinePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
      const Eigen::Vector3f &X, Eigen::Vector3f &unit_direction);

  bool getCornerFeatureCoefficients(const PointType &A, const PointType &B,
      const PointType &X, int iterration, PointType &coeff);

  float getSurfacePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C,
      const Eigen::Vector3f &X, Eigen::Vector3f &surfNormal);

  bool getSurfaceFeatureCoefficients(const PointType &A, const PointType &B, const PointType &C,
      const PointType &X, int iterration, PointType &coefficients);
};

#endif
