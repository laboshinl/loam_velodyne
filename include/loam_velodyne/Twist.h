#ifndef LOAM_TWIST_H
#define LOAM_TWIST_H

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include "Angle.h"
#include "Vector3.h"


namespace loam {


/** \brief Twist composed by three angles and a three-dimensional position.
 *
 */
class Twist {
public:
  Twist()
        : rot_x(),
          rot_y(),
          rot_z(),
          pos() {}

  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;
};

inline void convertTo( const Twist& src, geometry_msgs::Twist& dst)
{
  dst.angular.x = src.rot_x.rad();
  dst.angular.y = src.rot_y.rad();
  dst.angular.z = src.rot_z.rad();

  dst.linear.x = src.pos.x();
  dst.linear.y = src.pos.y();
  dst.linear.z = src.pos.z();
}

inline void convertTo( const geometry_msgs::Twist& src, Twist& dst)
{
  dst.rot_x = src.angular.x;
  dst.rot_y = src.angular.y;
  dst.rot_z = src.angular.z;

  dst.pos.x() = src.linear.x;
  dst.pos.y() = src.linear.y;
  dst.pos.z() = src.linear.z;
}

inline void convertTo( const geometry_msgs::Pose& src, Twist& dst)
{
  //TODO, please explain why the signs of quaternion are changed
  double roll, pitch, yaw;
  const geometry_msgs::Quaternion& quat = src.orientation;
  tf::Matrix3x3( tf::Quaternion(quat.z, -quat.x, -quat.y, quat.w) ).getRPY(roll, pitch, yaw);

  dst.rot_x = -pitch;
  dst.rot_y = -yaw;
  dst.rot_z = roll;

  dst.pos.x() = src.position.x;
  dst.pos.y() = src.position.y;
  dst.pos.z() = src.position.z;
}

inline void convertTo( const Twist& src,
                       geometry_msgs::Pose& dst,
                       tf::StampedTransform& tf_trans)
{
  //TODO, please explain why the signs of quaternion are changed
  geometry_msgs::Quaternion quat =
      tf::createQuaternionMsgFromRollPitchYaw(src.rot_z.rad(),
                                              -src.rot_x.rad(),
                                              -src.rot_y.rad());
  dst.orientation.x = -quat.y;
  dst.orientation.y = -quat.z;
  dst.orientation.z = quat.x;
  dst.orientation.w = quat.w;
  dst.position.x = src.pos.x();
  dst.position.y = src.pos.y();
  dst.position.z = src.pos.z();

  tf_trans.setRotation(tf::Quaternion(-quat.y, -quat.z, quat.x, quat.w));
  tf_trans.setOrigin(tf::Vector3( src.pos.x(), src.pos.y(), src.pos.z()) );

}



} // end namespace loam

#endif //LOAM_TWIST_H
