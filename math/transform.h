#ifndef ROBOTICS_COMMON_MATH_TRANSFORM_H_
#define ROBOTICS_COMMON_MATH_TRANSFORM_H_

#include "robotics-common/math/unit_quaternion.h"
#include "robotics-common/math/vector3d.h"

namespace robotics_common {
namespace math {

class Transform {
 public:
  Transform(const UnitQuaternion& angular, const Vector3d& linear);

  Transform(const Transform& other) = default;
  Transform& operator=(const Transform&) = default;

  static const Transform& Identity();

  const UnitQuaternion& angular() const { return angular_; }

  void set_angular(const UnitQuaternion& angular) { angular_ = angular; }

  const Vector3d& linear() const { return linear_; }

  void set_linear(const Vector3d& linear) { linear_ = linear; }

  Transform Prepend(const Transform& prev) const;
  Transform Append(const Transform& other) const;

  Transform operator~() const;
  Transform operator*(const Transform& other) const {
    return this->Append(other);
  }

  bool GeometricallyEquals(const Transform& other, Epsilon angular_epsilon,
                           Epsilon linear_epsilon);

 private:
  UnitQuaternion angular_;
  Vector3d linear_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_TRANSFORM_H_
