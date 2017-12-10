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

#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H


#include "Twist.h"
#include "nanoflann_pcl.h"

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


namespace loam {

/** \brief Implementation of the LOAM laser odometry component.
 *
 */
class LaserOdometry {
public:
  explicit LaserOdometry(const float& scanPeriod = 0.1,
                         const uint16_t& ioRatio = 2,
                         const size_t& maxIterations = 25);

  /** \brief Setup component.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle& node,
                     ros::NodeHandle& privateNode);

  /** \brief Handler method for a new sharp corner cloud.
   *
   * @param cornerPointsSharpMsg the new sharp corner cloud message
   */
  void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg);

  /** \brief Handler method for a new less sharp corner cloud.
   *
   * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
   */
  void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg);

  /** \brief Handler method for a new flat surface cloud.
   *
   * @param surfPointsFlatMsg the new flat surface cloud message
   */
  void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg);

  /** \brief Handler method for a new less flat surface cloud.
   *
   * @param surfPointsLessFlatMsg the new less flat surface cloud message
   */
  void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg);

  /** \brief Handler method for a new full resolution cloud.
   *
   * @param laserCloudFullResMsg the new full resolution cloud message
   */
  void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

  /** \brief Handler method for a new IMU transformation information.
   *
   * @param laserCloudFullResMsg the new IMU transformation information message
   */
  void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg);

  /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
  void spin();

  /** \brief Try to process buffered data. */
  void process();


protected:
  /** \brief Reset flags, etc. */
  void reset();

  /** \brief Check if all required information for a new processing step is available. */
  bool hasNewData();

  /** \brief Transform the given point to the start of the sweep.
   *
   * @param pi the point to transform
   * @param po the point instance for storing the result
   */
  void transformToStart(const pcl::PointXYZI& pi,
                        pcl::PointXYZI& po);

  /** \brief Transform the given point cloud to the end of the sweep.
   *
   * @param cloud the point cloud to transform
   */
  size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  void pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                         const Angle& blx, const Angle& bly, const Angle& blz,
                         const Angle& alx, const Angle& aly, const Angle& alz,
                         Angle &acx, Angle &acy, Angle &acz);

  void accumulateRotation(Angle cx, Angle cy, Angle cz,
                          Angle lx, Angle ly, Angle lz,
                          Angle &ox, Angle &oy, Angle &oz);

  /** \brief Publish the current result via the respective topics. */
  void publishResult();

private:
  float _scanPeriod;       ///< time per scan
  uint16_t _ioRatio;       ///< ratio of input to output frames
  bool _systemInited;      ///< initialization flag
  long _frameCount;        ///< number of processed frames

  size_t _maxIterations;   ///< maximum number of iterations
  float _deltaTAbort;     ///< optimization abort threshold for deltaT
  float _deltaRAbort;     ///< optimization abort threshold for deltaR

  pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat;         ///< flat surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud

  pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud;    ///< last corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud;   ///< last surface points cloud

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;      ///< point selection
  pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;           ///< point selection coefficients

  nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree
  nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

  ros::Time _timeCornerPointsSharp;      ///< time of current sharp corner cloud
  ros::Time _timeCornerPointsLessSharp;  ///< time of current less sharp corner cloud
  ros::Time _timeSurfPointsFlat;         ///< time of current flat surface cloud
  ros::Time _timeSurfPointsLessFlat;     ///< time of current less flat surface cloud
  ros::Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
  ros::Time _timeImuTrans;               ///< time of current IMU transformation information

  bool _newCornerPointsSharp;       ///< flag if a new sharp corner cloud has been received
  bool _newCornerPointsLessSharp;   ///< flag if a new less sharp corner cloud has been received
  bool _newSurfPointsFlat;          ///< flag if a new flat surface cloud has been received
  bool _newSurfPointsLessFlat;      ///< flag if a new less flat surface cloud has been received
  bool _newLaserCloudFullRes;       ///< flag if a new full resolution cloud has been received
  bool _newImuTrans;                ///< flag if a new IMU transformation information cloud has been received

  std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
  std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

  std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
  std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
  std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

  Twist _transform;     ///< optimized pose transformation
  Twist _transformSum;  ///< accumulated optimized pose transformation

  Angle _imuRollStart, _imuPitchStart, _imuYawStart;
  Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;

  Vector3 _imuShiftFromStart;
  Vector3 _imuVeloFromStart;

  nav_msgs::Odometry _laserOdometryMsg;       ///< laser odometry message
  tf::StampedTransform _laserOdometryTrans;   ///< laser odometry transformation

  ros::Publisher _pubLaserCloudCornerLast;  ///< last corner cloud message publisher
  ros::Publisher _pubLaserCloudSurfLast;    ///< last surface cloud message publisher
  ros::Publisher _pubLaserCloudFullRes;     ///< full resolution cloud message publisher
  ros::Publisher _pubLaserOdometry;         ///< laser odometry publisher
  tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

  ros::Subscriber _subCornerPointsSharp;      ///< sharp corner cloud message subscriber
  ros::Subscriber _subCornerPointsLessSharp;  ///< less sharp corner cloud message subscriber
  ros::Subscriber _subSurfPointsFlat;         ///< flat surface cloud message subscriber
  ros::Subscriber _subSurfPointsLessFlat;     ///< less flat surface cloud message subscriber
  ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud message subscriber
  ros::Subscriber _subImuTrans;               ///< IMU transformation information message subscriber
};

} // end namespace loam

#endif //LOAM_LASERODOMETRY_H
