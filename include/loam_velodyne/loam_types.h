#ifndef LOAM_LOAM_TYPES_H
#define LOAM_LOAM_TYPES_H


#include <pcl/point_types.h>


namespace loam {

/** \brief Vector4f decorator for convenient handling.
 *
 */
class Vector3 : public Eigen::Vector4f {
public:
  Vector3(float x, float y, float z)
      : Eigen::Vector4f(x, y, z, 0) {}

  Vector3(void)
      : Eigen::Vector4f(0, 0, 0, 0) {}

  template<typename OtherDerived>
  Vector3(const Eigen::MatrixBase <OtherDerived> &other)
      : Eigen::Vector4f(other) {}

  Vector3(const pcl::PointXYZI &p)
      : Eigen::Vector4f(p.x, p.y, p.z, 0) {}

  template<typename OtherDerived>
  Vector3 &operator=(const Eigen::MatrixBase <OtherDerived> &rhs) {
    this->Eigen::Vector4f::operator=(rhs);
    return *this;
  }

  Vector3 &operator=(const pcl::PointXYZ &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  Vector3 &operator=(const pcl::PointXYZI &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  float x() const { return (*this)(0); }

  float y() const { return (*this)(1); }

  float z() const { return (*this)(2); }

  float &x() { return (*this)(0); }

  float &y() { return (*this)(1); }

  float &z() { return (*this)(2); }

  // easy conversion
  operator pcl::PointXYZI() {
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
class Angle {
public:
  Angle()
      : _radian(0.0),
        _cos(1.0),
        _sin(0.0) {}

  Angle(float radValue)
      : _radian(radValue),
        _cos(std::cos(radValue)),
        _sin(std::sin(radValue)) {}

  Angle(const Angle &other)
      : _radian(other._radian),
        _cos(other._cos),
        _sin(other._sin) {}

  void operator=(const Angle &rhs) {
    _radian = (rhs._radian);
    _cos = (rhs._cos);
    _sin = (rhs._sin);
  }

  void operator+=(const float &radValue) { *this = (_radian + radValue); }

  void operator+=(const Angle &other) { *this = (_radian + other._radian); }

  void operator-=(const float &radValue) { *this = (_radian - radValue); }

  void operator-=(const Angle &other) { *this = (_radian - other._radian); }

  Angle operator-() const {
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

} // end namespace loam

#endif //LOAM_LOAM_TYPES_H
