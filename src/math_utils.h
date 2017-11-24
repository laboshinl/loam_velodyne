#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <pcl/point_types.h>


class Vector3 : public Eigen::Vector4f
{
public:

    Vector3(float x, float y, float z):Eigen::Vector4f(x,y,z,0) {}

    Vector3(void):Eigen::Vector4f(0,0,0,0) {}

    template<typename OtherDerived>
    Vector3(const Eigen::MatrixBase<OtherDerived>& other)
        : Eigen::Vector4f(other)
    { }

    template<typename OtherDerived>
    Vector3& operator=(const Eigen::MatrixBase <OtherDerived>& other){
        this->Eigen::Vector4f::operator=(other);
        return *this;
    }

    Vector3(const pcl::PointXYZI& p):Eigen::Vector4f( p.x, p.y, p.z,0) {}

    Vector3& operator=(const pcl::PointXYZI& point) {
        x() = point.x;
        y() = point.y;
        z() = point.z;
        return *this;
    }

    float x() const { return (*this)(0); }
    float y() const { return (*this)(1); }
    float z() const { return (*this)(2); }

    float& x() { return (*this)(0); }
    float& y() { return (*this)(1); }
    float& z() { return (*this)(2); }
};

class Angle{
public:
    Angle():
        _ang(0.0),
        _cos(1.0),
        _sin(0.0) {}

    Angle(float value):
        _ang(value),
        _cos(std::cos(value)),
        _sin(std::sin(value)) {}

    Angle( const Angle& other ):
        _ang( other._ang ),
        _cos( other._cos ),
        _sin( other._sin ) {}

    void operator =( const Angle& other){
        _ang = ( other._ang );
        _cos = ( other._cos );
        _sin = ( other._sin );
    }

    void operator +=( const float& val)   { *this = ( _ang + val) ; }

    void operator +=( const Angle& other) { *this = ( _ang + other._ang ); }

    void operator -=( const float& val)   { *this = ( _ang - val ); }

    void operator -=( const Angle& other) { *this = ( _ang - other._ang ); }

    Angle operator-() const
    {
        Angle out;
        out._ang = _ang;
        out._cos = _cos;
        out._sin = -(_sin);
        return out;
    }

    float value() const { return _ang; }

    float cos() const { return _cos; }

    float sin() const { return _sin; }

private:
    float _ang, _cos, _sin;
};


inline Vector3 rotateX(const Vector3& v,const Angle& ang)
{
    return Vector3( v.x(),
                    ang.cos() * v.y() - ang.sin() * v.z(),
                    ang.sin() * v.y() + ang.cos() * v.z() );
}

inline Vector3 rotateY(const Vector3& v,const Angle& ang)
{
    return Vector3( ang.cos() * v.x() + ang.sin() * v.z(),
                    v.y(),
                   -ang.sin() * v.x() + ang.cos() * v.z() );
}

inline Vector3 rotateZ(const Vector3& v,const Angle& ang)
{
    return Vector3( ang.cos() * v.x() - ang.sin() * v.y(),
                    ang.sin() * v.x() + ang.cos() * v.y(),
                    v.z() );
}

struct Twist{
  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;
};


#endif // MATH_UTILS_H
