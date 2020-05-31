#ifndef ROBOTICS_COMMON_MATH_VECTOR3D_H_
#define ROBOTICS_COMMON_MATH_VECTOR3D_H_

#include "robotics-common/math/const_vector3d.h"

namespace robotics_common {
namespace math {

class Vector3d : public ConstVector3d<Vector3d> {
 public:
  Vector3d(double x = 0.0, double y = 0.0, double z = 0.0);

  Vector3d(const Vector3d&) = default;
  Vector3d& operator=(const Vector3d&) = default;

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  void set_z(double z) { z_ = z; }

  void Set(Axis3d axis, double value);

  Vector3d operator*(double scalar) const {
    return ToBase()->operator*<Vector3d>(scalar);
  }

  template <typename U>
  Vector3d operator+(const ConstVector3d<U>& other) const {
    return ToBase()->operator+<Vector3d, U>(other);
  }

  template <typename U>
  Vector3d Cross(const ConstVector3d<U>& other) const {
    return ToBase()->Cross<Vector3d, U>(other);
  }

  void operator*=(double scalar) { *this = *this * scalar; }

  template <typename U>
  void operator+=(const ConstVector3d<U>& other) {
    *this = *this + other;
  }

  template <typename U>
  void operator-=(const ConstVector3d<U>& other) {
    *this = *this - other;
  }

  Vector3d Rotate(const UnitQuaternion& rotation) const {
    return ToBase()->Rotate<Vector3d>(rotation);
  }

 private:
  ConstVector3d<Vector3d> const* ToBase() const {
    return static_cast<ConstVector3d<Vector3d> const*>(this);
  }

  double x_;
  double y_;
  double z_;
};

template <typename U>
inline Vector3d operator*(double scalar, const ConstVector3d<U>& vector) {
  return vector.template operator*<Vector3d>(scalar);
}

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_VECTOR3D_H_
