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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H


#include "Twist.h"
#include "CircularBuffer.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


namespace loam {

/** IMU state data. */
typedef struct IMUState2 {
  /** The time of the measurement leading to this state (in seconds). */
  ros::Time stamp;

  /** The current roll angle. */
  Angle roll;

  /** The current pitch angle. */
  Angle pitch;

  /** \brief Interpolate between two IMU states.
   *
   * @param start the first IMU state
   * @param end the second IMU state
   * @param ratio the interpolation ratio
   * @param result the target IMU state for storing the interpolation result
   */
  static void interpolate(const IMUState2& start,
                          const IMUState2& end,
                          const float& ratio,
                          IMUState2& result)
  {
    float invRatio = 1 - ratio;

    result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
    result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
  };
} IMUState2;



/** \brief Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping {
public:
  explicit LaserMapping(const float& scanPeriod = 0.1,
                        const size_t& maxIterations = 10);

  /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle& node,
                     ros::NodeHandle& privateNode);

  /** \brief Handler method for a new last corner cloud.
   *
   * @param cornerPointsLastMsg the new last corner cloud message
   */
  void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg);

  /** \brief Handler method for a new last surface cloud.
   *
   * @param surfacePointsLastMsg the new last surface cloud message
   */
  void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg);

  /** \brief Handler method for a new full resolution cloud.
   *
   * @param laserCloudFullResMsg the new full resolution cloud message
   */
  void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

  /** \brief Handler method for a new laser odometry.
   *
   * @param laserOdometry the new laser odometry message
   */
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

  /** \brief Handler method for IMU messages.
   *
   * @param imuIn the new IMU message
   */
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);

  /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
  void spin();

  /** \brief Try to process buffered data. */
  void process();


protected:
  /** \brief Reset flags, etc. */
  void reset();

  /** \brief Check if all required information for a new processing step is available. */
  bool hasNewData();

  /** Run an optimization. */
  void optimizeTransformTobeMapped();

  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
  void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);

  /** \brief Publish the current result via the respective topics. */
  void publishResult();


private:

  size_t toIndex(int i, int j, int k) const
  {
    return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
  }


  float _scanPeriod;          ///< time per scan
  const int _stackFrameNum;
  const int _mapFrameNum;
  long _frameCount;
  long _mapFrameCount;

  size_t _maxIterations;  ///< maximum number of iterations
  float _deltaTAbort;     ///< optimization abort threshold for deltaT
  float _deltaRAbort;     ///< optimization abort threshold for deltaR

  int _laserCloudCenWidth;
  int _laserCloudCenHeight;
  int _laserCloudCenDepth;
  const size_t _laserCloudWidth;
  const size_t _laserCloudHeight;
  const size_t _laserCloudDepth;
  const size_t _laserCloudNum;

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;   ///< last corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;     ///< last surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

  std::vector<size_t> _laserCloudValidInd;
  std::vector<size_t> _laserCloudSurroundInd;

  ros::Time _timeLaserCloudCornerLast;   ///< time of current last corner cloud
  ros::Time _timeLaserCloudSurfLast;     ///< time of current last surface cloud
  ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
  ros::Time _timeLaserOdometry;          ///< time of current laser odometry

  bool _newLaserCloudCornerLast;  ///< flag if a new last corner cloud has been received
  bool _newLaserCloudSurfLast;    ///< flag if a new last surface cloud has been received
  bool _newLaserCloudFullRes;     ///< flag if a new full resolution cloud has been received
  bool _newLaserOdometry;         ///< flag if a new laser odometry has been received

  Twist _transformSum;
  Twist _transformIncre;
  Twist _transformTobeMapped;
  Twist _transformBefMapped;
  Twist _transformAftMapped;

  CircularBuffer<IMUState2> _imuHistory;    ///< history of IMU states

  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;   ///< voxel filter for down sizing corner clouds
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;     ///< voxel filter for down sizing surface clouds
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMap;      ///< voxel filter for down sizing accumulated map

  nav_msgs::Odometry _odomAftMapped;      ///< mapping odometry message
  tf::StampedTransform _aftMappedTrans;   ///< mapping odometry transformation

  ros::Publisher _pubLaserCloudSurround;    ///< map cloud message publisher
  ros::Publisher _pubLaserCloudFullRes;     ///< current full resolution cloud message publisher
  ros::Publisher _pubOdomAftMapped;         ///< mapping odometry publisher
  tf::TransformBroadcaster _tfBroadcaster;  ///< mapping odometry transform broadcaster

  ros::Subscriber _subLaserCloudCornerLast;   ///< last corner cloud message subscriber
  ros::Subscriber _subLaserCloudSurfLast;     ///< last surface cloud message subscriber
  ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud message subscriber
  ros::Subscriber _subLaserOdometry;          ///< laser odometry message subscriber
  ros::Subscriber _subImu;                    ///< IMU message subscriber

};

} // end namespace loam

#endif //LOAM_LASERMAPPING_H
