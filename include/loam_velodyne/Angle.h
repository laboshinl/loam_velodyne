#ifndef LOAM_ANGLE_H
#define LOAM_ANGLE_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>


namespace loam {


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

  float deg() const { return float(_radian * 180 / M_PI); }

  float cos() const { return _cos; }

  float sin() const { return _sin; }

private:
  float _radian;    ///< angle value in radian
  float _cos;       ///< cosine of the angle
  float _sin;       ///< sine of the angle
};

} // end namespace loam

#endif //LOAM_ANGLE_H
