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

#include "ScanRegistration.h"

#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>


namespace loam {

ScanRegistration::ScanRegistration(const float& scanPeriod,
                                   const uint16_t& nScans,
                                   const size_t& imuHistorySize,
                                   const RegistrationParams& config)
      : _nScans(nScans),
        _scanPeriod(scanPeriod),
        _sweepStamp(),
        _config(config),
        _imuStart(),
        _imuCur(),
        _imuStartIdx(0),
        _imuHistory(imuHistorySize),
        _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
        _cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
        _cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
        _surfacePointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
        _surfacePointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
        _imuTrans(new pcl::PointCloud<pcl::PointXYZ>(4, 1)),
        _regionCurvature(),
        _regionLabel(),
        _regionSortIndices(),
        _scanNeighborPicked(),
        _activeMode(false)
{
  _scanStartIndices.assign(nScans, 0);
  _scanEndIndices.assign(nScans, 0);
};



bool ScanRegistration::setup(ros::NodeHandle& node,
                             ros::NodeHandle& privateNode)
{
  if (!_config.parseParams(privateNode)) {
    return false;
  }
  _config.print();

  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);


  // advertise scan registration topics
  _pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  // set active mode
  _activeMode = true;

  return true;
}



bool ScanRegistration::setup(const RegistrationParams& config)
{
  _config = config;
  _config.print();

  return true;
}



void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  IMUState newState;
  newState.stamp = imuIn->header.stamp;
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    const IMUState& prevState = _imuHistory.last();
    float timeDiff = float((newState.stamp - prevState.stamp).toSec());
    newState.position = prevState.position
                        + (prevState.velocity * timeDiff)
                        + (0.5 * acc * timeDiff * timeDiff);
    newState.velocity = prevState.velocity
                        + acc * timeDiff;
  }

  _imuHistory.push(newState);
}



void ScanRegistration::reset(const ros::Time& scanTime)
{
  _sweepStamp = scanTime;

  // clear cloud buffers
  _laserCloud->clear();
  _cornerPointsSharp->clear();
  _cornerPointsLessSharp->clear();
  _surfacePointsFlat->clear();
  _surfacePointsLessFlat->clear();


  // reset scan indices vectors
  _scanStartIndices.assign(_nScans, 0);
  _scanEndIndices.assign(_nScans, 0);


  // re-initialize IMU start state and index
  _imuStartIdx = 0;

  if (_imuHistory.size() > 0) {
    while (_imuStartIdx < _imuHistory.size() - 1 && (scanTime - _imuHistory[_imuStartIdx].stamp).toSec() > 0) {
      _imuStartIdx++;
    }

    // fetch / interpolate IMU start state
    if (_imuStartIdx == 0 || (scanTime - _imuHistory[_imuStartIdx].stamp).toSec() > 0) {
      // scan time newer then newest or older than oldest IMU message
      _imuStart = _imuHistory[_imuStartIdx];
    } else {
      float ratio = (_imuHistory[_imuStartIdx].stamp - scanTime).toSec()
                    / (_imuHistory[_imuStartIdx].stamp - _imuHistory[_imuStartIdx - 1].stamp).toSec();
      IMUState::interpolate(_imuHistory[_imuStartIdx], _imuHistory[_imuStartIdx - 1], ratio, _imuStart);
    }
  }
}



void ScanRegistration::transformToStartIMU(pcl::PointXYZI& point,
                                           const float& pointTime)
{
  // rotate point to global IMU system
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift
  Vector3 positionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * pointTime;
  point.x += positionShift.x();
  point.y += positionShift.y();
  point.z += positionShift.z();

  // rotate point back to local IMU system relative to the start IMU state
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}



void ScanRegistration::setIMUTrans(const double& sweepDuration)
{
  _imuTrans->points[0].x = _imuStart.pitch.rad();
  _imuTrans->points[0].y = _imuStart.yaw.rad();
  _imuTrans->points[0].z = _imuStart.roll.rad();

  _imuTrans->points[1].x = _imuCur.pitch.rad();
  _imuTrans->points[1].y = _imuCur.yaw.rad();
  _imuTrans->points[1].z = _imuCur.roll.rad();

  Vector3 imuShiftFromStart = _imuCur.position - _imuStart.position - _imuStart.velocity * sweepDuration;
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans->points[2].x = imuShiftFromStart.x();
  _imuTrans->points[2].y = imuShiftFromStart.y();
  _imuTrans->points[2].z = imuShiftFromStart.z();

  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans->points[3].x = imuVelocityFromStart.x();
  _imuTrans->points[3].y = imuVelocityFromStart.y();
  _imuTrans->points[3].z = imuVelocityFromStart.z();
}



void ScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  // extract features from individual scans
  for (int i = beginIdx; i < _nScans; i++) {
    // ROS_INFO("Extract features for scan %d", i);

    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
    size_t scanStartIdx = _scanStartIndices[i];
    size_t scanEndIdx = _scanEndIndices[i];

    // skip empty scans
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
      continue;
    }

    // Quick&Dirty fix for relative point time calculation without IMU data
    /*float scanSize = scanEndIdx - scanStartIdx + 1;
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {
      _laserCloud->points[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
    }*/

    // reset scan buffers
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions
    for (int j = 0; j < _config.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j)
                   + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;

      // reset region buffers
      setRegionBuffersFor(sp, ep);


      // ROS_INFO("Extract corner features");

      // extract corner features
      int largestPickedNum = 0;
      for (size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;) {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold) {

          largestPickedNum++;
          if (largestPickedNum <= _config.maxCornerSharp) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp->push_back(_laserCloud->points[idx]);
            _cornerPointsLessSharp->push_back(_laserCloud->points[idx]);
          } else {
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
            _cornerPointsLessSharp->push_back(_laserCloud->points[idx]);
          }

          _scanNeighborPicked[scanIdx] = 1;
          for (int l = 1; l <= _config.curvatureRegion; l++) {
            float distSquare = calcSquaredDiff(_laserCloud->points[idx + l], _laserCloud->points[idx + l - 1]);
            if (distSquare > 0.05) {
              break;
            }

            _scanNeighborPicked[scanIdx + l] = 1;
          }
          for (int l = 1; l <= _config.curvatureRegion; l++) {
            float distSquare = calcSquaredDiff(_laserCloud->points[idx - l], _laserCloud->points[idx - l + 1]);
            if (distSquare > 0.05) {
              break;
            }

            _scanNeighborPicked[scanIdx - l] = 1;
          }
        }
      }

      // ROS_INFO("Extract flat features");

      // extract flat surface features
      int smallestPickedNum = 0;
      for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++) {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {

          smallestPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat->push_back(_laserCloud->points[idx]);

          _scanNeighborPicked[scanIdx] = 1;
          for (int l = 1; l <= _config.curvatureRegion; l++) {
            float distSquare = calcSquaredDiff(_laserCloud->points[idx + l], _laserCloud->points[idx + l - 1]);
            if (distSquare > 0.05) {
              break;
            }

            _scanNeighborPicked[scanIdx + l] = 1;
          }
          for (int l = 1; l <= _config.curvatureRegion; l++) {
            float distSquare = calcSquaredDiff(_laserCloud->points[idx - l], _laserCloud->points[idx - l + 1]);
            if (distSquare > 0.05) {
              break;
            }

            _scanNeighborPicked[scanIdx - l] = 1;
          }
        }
      }

      // ROS_INFO("Extract less flat features");

      // extract less flat surface features
      for (int k = 0; k < regionSize; k++) {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
          surfPointsLessFlatScan->push_back(_laserCloud->points[sp + k]);
        }
      }
    }

    // down size less flat surface point cloud of current scan
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *_surfacePointsLessFlat += surfPointsLessFlatScanDS;
  }
}



void ScanRegistration::setRegionBuffersFor(const size_t& startIdx,
                                           const size_t& endIdx)
{
  // ROS_INFO("Set region buffers for %d  to  %d", int(startIdx), int(endIdx));

  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * _config.curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
    float diffX = pointWeight * _laserCloud->points[i].x;
    float diffY = pointWeight * _laserCloud->points[i].y;
    float diffZ = pointWeight * _laserCloud->points[i].z;

    for (int j = 1; j <= _config.curvatureRegion; j++) {
      diffX += _laserCloud->points[i + j].x + _laserCloud->points[i - j].x;
      diffY += _laserCloud->points[i + j].y + _laserCloud->points[i - j].y;
      diffZ += _laserCloud->points[i + j].z + _laserCloud->points[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i;
  }

  // sort point curvatures
  for (size_t i = 1; i < regionSize; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }
}



void ScanRegistration::setScanBuffersFor(const size_t& startIdx,
                                         const size_t& endIdx)
{
  // ROS_INFO("Set scan buffers for %d  to  %d", int(startIdx), int(endIdx));

  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked
  for (size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++) {
    const PointType& prevPoint = (_laserCloud->points[i - 1]);
    const PointType& point = (_laserCloud->points[i]);
    const PointType& nextPoint = (_laserCloud->points[i + 1]);

    float diff = calcSquaredDiff(nextPoint, point);

    if (diff > 0.1) {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);

      if (depth1 > depth2) {
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1) {
          size_t scanIdx = i - startIdx;
          for (int j = 0; j <= _config.curvatureRegion; j++) {
            _scanNeighborPicked[scanIdx - j] = 1;
          }

          continue;
        }
      } else {
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1) {
          size_t scanIdx = i - startIdx;
          for (int j = _config.curvatureRegion + 1; j > 0 ; j--) {
            _scanNeighborPicked[scanIdx + j] = 1;
          }
        }
      }
    }

    float diff2 = calcSquaredDiff(point, prevPoint);
    float dis = calcSquaredPointDistance(point);

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}



void ScanRegistration::generateROSMsg(sensor_msgs::PointCloud2& msg,
                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = _sweepStamp;
  msg.header.frame_id = "/camera";
}



void ScanRegistration::publishResult()
{
  if (!_activeMode) {
    // only publish messages in active mode
    return;
  }

  // publish full resolution and feature point clouds
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  generateROSMsg(laserCloudOutMsg, _laserCloud);
  _pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  generateROSMsg(cornerPointsSharpMsg, _cornerPointsSharp);
  _pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  generateROSMsg(cornerPointsLessSharpMsg, _cornerPointsLessSharp);
  _pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat;
  generateROSMsg(surfPointsFlat, _surfacePointsFlat);
  _pubSurfPointsFlat.publish(surfPointsFlat);

  sensor_msgs::PointCloud2 surfPointsLessFlat;
  generateROSMsg(surfPointsLessFlat, _surfacePointsLessFlat);
  _pubSurfPointsLessFlat.publish(surfPointsLessFlat);


  // publish corresponding IMU transformation information
  sensor_msgs::PointCloud2 imuTransMsg;
  pcl::toROSMsg(*_imuTrans, imuTransMsg);
  imuTransMsg.header.stamp = _sweepStamp;
  imuTransMsg.header.frame_id = "/camera";
  _pubImuTrans.publish(imuTransMsg);
}

} // end namespace loam
