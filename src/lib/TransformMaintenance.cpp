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

#include "loam_velodyne/TransformMaintenance.h"
#include "math_utils.h"

namespace loam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;


TransformMaintenance::TransformMaintenance()
{
  // initialize odometry and odometry tf messages
  _laserOdometry2.header.frame_id = "/camera_init";
  _laserOdometry2.child_frame_id = "/camera";

  _laserOdometryTrans2.frame_id_ = "/camera_init";
  _laserOdometryTrans2.child_frame_id_ = "/camera";
}


bool TransformMaintenance::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
{
  // advertise integrated laser odometry topic
  _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);

  // subscribe to laser odometry and mapping odometry topics
  _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &TransformMaintenance::laserOdometryHandler, this);

  _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &TransformMaintenance::odomAftMappedHandler, this);

  return true;
}



void TransformMaintenance::transformAssociateToMap()
{
  Vector3 v1 = _transformBefMapped.pos - _transformSum.pos;
  rotateYXZ( v1,  -_transformSum.rot_y,   -_transformSum.rot_x,  -_transformSum.rot_z );

  _transformIncre.pos = v1;

  float sbcx = _transformSum.rot_x.sin();
  float cbcx = _transformSum.rot_x.cos();
  float sbcy = _transformSum.rot_y.sin();
  float cbcy = _transformSum.rot_y.cos();
  float sbcz = _transformSum.rot_z.sin();
  float cbcz = _transformSum.rot_z.cos();

  float sblx = _transformBefMapped.rot_x.sin();
  float cblx = _transformBefMapped.rot_x.cos();
  float sbly = _transformBefMapped.rot_y.sin();
  float cbly = _transformBefMapped.rot_y.cos();
  float sblz = _transformBefMapped.rot_z.sin();
  float cblz = _transformBefMapped.rot_z.cos();

  float salx = _transformAftMapped.rot_x.sin();
  float calx = _transformAftMapped.rot_x.cos();
  float saly = _transformAftMapped.rot_y.sin();
  float caly = _transformAftMapped.rot_y.cos();
  float salz = _transformAftMapped.rot_z.sin();
  float calz = _transformAftMapped.rot_z.cos();

  float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
      - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                   - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                   + cblx*salx*sbly)
      - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                   - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                   + cblx*cbly*salx);
  _transformMapped.rot_x = -asin(srx);
  float cos_transformMapped_x = _transformMapped.rot_x.cos();

  float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                       - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
      - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                   + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx)
                   - calx*cblx*cbly*saly)
      + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                   + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                   + calx*cblx*saly*sbly);
  float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                       - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
      + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                   + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                   + calx*caly*cblx*cbly)
      - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                   + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz)
                   - calx*caly*cblx*sbly);
  _transformMapped.rot_y = atan2(srycrx / cos_transformMapped_x,
                                 crycrx / cos_transformMapped_x);

  float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                 - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                                               + cblx*cbly*salx)
      - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                      - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                                      + cblx*salx*sbly)
      + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                               - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                                               + cblx*salx*sbly)
      - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                      - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                                      + cblx*cbly*salx)
      + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  _transformMapped.rot_z = atan2(srzcrx / cos_transformMapped_x,
                                 crzcrx / cos_transformMapped_x);

  Vector3 v2 = _transformIncre.pos;
  rotateZXY( v2, _transformMapped.rot_z, _transformMapped.rot_y, _transformMapped.rot_x  );
  _transformMapped.pos = _transformAftMapped.pos - v2;

}

void TransformMaintenance::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  convertTo( laserOdometry->pose.pose, _transformSum);
  transformAssociateToMap();
  convertTo( _transformMapped, _laserOdometry2.pose.pose, _laserOdometryTrans2);

  _laserOdometry2.header.stamp = laserOdometry->header.stamp;
  _pubLaserOdometry2.publish(_laserOdometry2);

  _laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
  _tfBroadcaster2.sendTransform(_laserOdometryTrans2);
}



void TransformMaintenance::odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
  convertTo( odomAftMapped->pose.pose, _transformAftMapped);
  convertTo( odomAftMapped->twist.twist, _transformBefMapped );

}

} // end namespace loam
