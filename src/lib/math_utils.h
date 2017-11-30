#ifndef LOAM_MATH_UTILS_H
#define LOAM_MATH_UTILS_H

#include <cmath>
#include <pcl/point_types.h>


namespace loam {

/** \brief Vector4f decorator for convenient handling.
 *
 */
class Vector3 : public Eigen::Vector4f
{
public:
  Vector3(float x, float y, float z)
      : Eigen::Vector4f(x, y, z, 0) {}

  Vector3(void)
      : Eigen::Vector4f(0, 0, 0, 0) {}

  template<typename OtherDerived>
  Vector3(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Vector4f(other) {}

  Vector3(const pcl::PointXYZI& p)
      : Eigen::Vector4f(p.x, p.y, p.z, 0) {}

  template<typename OtherDerived>
  Vector3& operator=(const Eigen::MatrixBase <OtherDerived>& rhs) {
    this->Eigen::Vector4f::operator=(rhs);
    return *this;
  }

  Vector3& operator=(const pcl::PointXYZ& rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  Vector3& operator=(const pcl::PointXYZI& rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  float x() const { return (*this)(0); }
  float y() const { return (*this)(1); }
  float z() const { return (*this)(2); }

  float& x() { return (*this)(0); }
  float& y() { return (*this)(1); }
  float& z() { return (*this)(2); }

  // easy conversion
  operator pcl::PointXYZI()
  {
      pcl::PointXYZI dst;
      dst.x = x();
      dst.y = y();
      dst.z = z();
      dst.intensity = 0;
      return dst;
  }
};



/** \brief Class for holding an angle.
 *
 * This class provides buffered access to sine and cosine values to the represented angular value.
 */
class Angle{
public:
  Angle()
      : _radian(0.0),
        _cos(1.0),
        _sin(0.0) {}

  Angle(float radValue)
      : _radian(radValue),
        _cos(std::cos(radValue)),
        _sin(std::sin(radValue)) {}

  Angle(const Angle& other)
      : _radian(other._radian),
        _cos(other._cos),
        _sin(other._sin) {}

  void operator =(const Angle& rhs){
      _radian = (rhs._radian);
      _cos = (rhs._cos);
      _sin = (rhs._sin);
  }

  void operator +=( const float& radValue)   { *this = ( _radian + radValue) ; }

  void operator +=( const Angle& other) { *this = ( _radian + other._radian ); }

  void operator -=( const float& radValue)   { *this = ( _radian - radValue ); }

  void operator -=( const Angle& other) { *this = ( _radian - other._radian ); }

  Angle operator-() const
  {
      Angle out;
      out._radian = -_radian;
      out._cos = _cos;
      out._sin = -(_sin);
      return out;
  }

  float rad() const { return _radian; }

  float deg() const { return _radian * 180 / M_PI; }

  float cos() const { return _cos; }

  float sin() const { return _sin; }

private:
  float _radian;    ///< angle value in radian
  float _cos;       ///< cosine of the angle
  float _sin;       ///< sine of the angle
};



/** \brief Twist composed by three angles and a three-dimensional position. */
struct Twist {
  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;
};



/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}



/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians)
{
  return (float) (radians * 180.0 / M_PI);
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees)
{
  return (float) (degrees * M_PI / 180.0);
}




/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
inline float calcSquaredDiff(const pcl::PointXYZI& a, const pcl::PointXYZI& b)
{
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}



/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
inline float calcSquaredDiff(const pcl::PointXYZI& a, const pcl::PointXYZI& b, const float& wb)
{
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}


/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
inline float calcPointDistance(const pcl::PointXYZI& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}



/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
inline float calcSquaredPointDistance(const pcl::PointXYZI& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}



/** \brief Rotate the given vector by the specified angle around the x-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotX(Vector3& v, const Angle& ang)
{
  float y = v.y();
  v.y() = ang.cos() * y - ang.sin() * v.z();
  v.z() = ang.sin() * y + ang.cos() * v.z();
}

/** \brief Rotate the given point by the specified angle around the x-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
inline void rotX(pcl::PointXYZI& p, const Angle& ang)
{
  float y = p.y;
  p.y = ang.cos() * y - ang.sin() * p.z;
  p.z = ang.sin() * y + ang.cos() * p.z;
}



/** \brief Rotate the given vector by the specified angle around the y-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotY(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x + ang.sin() * v.z();
  v.z() = ang.cos() * v.z() - ang.sin() * x;
}

/** \brief Rotate the given point by the specified angle around the y-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
inline void rotY(pcl::PointXYZI& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x + ang.sin() * p.z;
  p.z = ang.cos() * p.z - ang.sin() * x;
}



/** \brief Rotate the given vector by the specified angle around the z-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotZ(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x - ang.sin() * v.y();
  v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given point by the specified angle around the z-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
inline void rotZ(pcl::PointXYZI& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x - ang.sin() * p.y;
  p.y = ang.sin() * x + ang.cos() * p.y;
}



/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(v, angZ);
  rotX(v, angX);
  rotY(v, angY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 *
 * @param p the point to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(pcl::PointXYZI& p,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(p, angZ);
  rotX(p, angX);
  rotY(p, angY);
}



/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 *
 * @param v the vector to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(v, angY);
  rotX(v, angX);
  rotZ(v, angZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 *
 * @param p the point to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(pcl::PointXYZI& p,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(p, angY);
  rotX(p, angX);
  rotZ(p, angZ);
}

} // end namespace loam


#endif // LOAM_MATH_UTILS_H
