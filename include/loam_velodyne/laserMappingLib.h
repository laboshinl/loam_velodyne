#ifndef __LOAM_LASER_MAPPING_H__
#define __LOAM_LASER_MAPPING_H__

#include <math.h>

#include <loam_velodyne/laserOdometryLib.h>
#include <loam_velodyne/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


template <typename PointT>
class PointCloudGrid {
public:
  PointCloudGrid(int gridWidth_, int gridHeight_, int gridDepth_) :
    gridWidth(gridWidth_),
    gridHeight(gridHeight_),
    gridDepth(gridDepth_),
    grid(gridWidth*gridHeight*gridDepth) {
    for(int i = 0; i < gridWidth*gridHeight*gridDepth; i++) {
      grid[i].reset(new pcl::PointCloud<PointT>);
    }
  }

  typename pcl::PointCloud<PointT>::Ptr& operator()(int i, int j, int k) {
    return grid[i + j*gridWidth + k*gridWidth*gridHeight];
  }

  const typename pcl::PointCloud<PointT>::Ptr& operator()(int i, int j, int k) const {
    return grid[i + j*gridWidth + k*gridWidth*gridHeight];
  }

  typename pcl::PointCloud<PointT>::Ptr& operator[](int i) {
    return grid[i];
  }

  const typename pcl::PointCloud<PointT>::Ptr& operator[](int i) const {
    return grid[i];
  }

  void shift(int dIndexI, int dIndexJ, int dIndexK) {
    //ROS_WARN("shift %d %d %d", dIndexI, dIndexJ, dIndexK);
    if(dIndexI != 0 || dIndexJ != 0 || dIndexK != 0) {
      for(int i = 0; i < gridWidth; i++) {
        for(int j = 0; j < gridHeight; j++) {
          for(int k = 0; k < gridDepth; k++) {
            int oldI = i - dIndexI;
            int oldJ = j - dIndexJ;
            int oldK = k - dIndexK;
            if(0 <= oldI && oldI < gridWidth && 0 <= oldJ && oldJ < gridHeight && 0 <= oldK && oldK < gridDepth) {
              (*this)(i, j, k).swap((*this)(oldI, oldJ, oldK));
            } else {
              (*this)(i, j, k)->clear();
            }
          }
        }
      }
    }
  }

private:
  int gridWidth, gridHeight, gridDepth;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> grid;
};

class LaserMapping {
private:
  float scanPeriod;

  static const int cloudGridWidth = 21;
  static const int cloudGridHeight = 11;
  static const int cloudGridDepth = 21;
  int laserCloudCenWidth;
  int laserCloudCenHeight;
  int laserCloudCenDepth;

  static const int VALID_PTS = 125;
  int laserCloudValidInd[VALID_PTS];
  int laserCloudSurroundInd[VALID_PTS];

  PointCloudGrid<PointType> laserCloudCornerArray;
  PointCloudGrid<PointType> laserCloudSurfArray;

  std::vector<float> transformTobeMapped;
  std::vector<float> transformBefMapped;
  std::vector<float> transformAftMapped;

  int imuPointerFront;
  int imuPointerLast;
  static const int imuQueLength = 200;

  double imuTime[imuQueLength];
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];

  bool isDegenerate;
  cv::Mat matP;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  int laserCloudSurroundNum;

public:
  class Inputs : public LaserOdometry::Outputs {};

  class Outputs {
  public:
    std::vector<float> transformToMap;
    std::vector<float> transformBeforeMapping;
    std::vector<float> transformAfterMapping;
    Outputs() :
      transformToMap(6, 0),
      transformBeforeMapping(6, 0),
      transformAfterMapping(6, 0) {
    }
  };

  LaserMapping(float scanPeriod_) :
    scanPeriod(scanPeriod_),
    laserCloudCenWidth(cloudGridWidth/2),
    laserCloudCenHeight(cloudGridHeight/2),
    laserCloudCenDepth(cloudGridDepth/2),
    laserCloudCornerArray(cloudGridWidth, cloudGridHeight , cloudGridDepth),
    laserCloudSurfArray(cloudGridWidth, cloudGridHeight , cloudGridDepth),
    transformTobeMapped(6, 0),
    transformBefMapped(6, 0),
    transformAftMapped(6, 0),
    imuPointerFront(0),
    imuPointerLast(-1),
    isDegenerate(false),
    matP(6, 6, CV_32F, cv::Scalar::all(0)),
    laserCloudSurroundNum(0)
  {
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

    std::fill(imuTime, &(imuTime[imuQueLength]), 0.0);
    std::fill(imuRoll, &(imuRoll[imuQueLength]), 0.0);
    std::fill(imuPitch, &(imuPitch[imuQueLength]), 0.0);
  }

  void run(const Inputs &inputs, Outputs &outputs, float timeLaserOdometry);

  pcl::PointCloud<PointType>::Ptr getSurroundingFeatures();

  void imuUpdate(float time, float roll, float pitch);

protected:
  void transformUpdate(float timeLaserOdometry, const std::vector<float> &laserOdomTransform);

  Eigen::Affine3f getAssociationToMap();

  void pointAssociateToMap(const PointType &pi, PointType &po);

  Eigen::Affine3f getAssociationToBeMapped();

  bool findPlane(const pcl::PointCloud<PointType> &cloud, const std::vector<int> &indices,
      float maxDistance, Eigen::Vector4f &coef);
};

#endif
