#ifndef ROBOTICS_COMMON_MATH_UNIT_QUATERNION_H_
#define ROBOTICS_COMMON_MATH_UNIT_QUATERNION_H_

#include "common/error_or.h"
#include "robotics-common/math/epsilons.h"
#include "robotics-common/math/geometry_defs.h"

namespace robotics_common {
namespace math {

// Note: Using a forward declaration here to resolve a circular dependency issue
// While this isn't ideal this enables UnitVector3d and UnitQuaternion to depend
// on each other as inputs.
class UnitVector3d;
class Vector3d;

class UnitQuaternion {
 public:
  constexpr static Epsilon kDefaultSmallAngleEpsilon = Epsilon::kTenMillionth;

  static const UnitQuaternion kIdentity;
  static common::ErrorOr<UnitQuaternion> Construct(double w, double x, double y,
                                                   double z, Epsilon epsilon);

  static UnitQuaternion FromAxisAngle(double angle, const UnitVector3d& axis);

  static UnitQuaternion FromTwist(
      const Vector3d& angular_twist,
      Epsilon small_angle_epsilon = kDefaultSmallAngleEpsilon);

  UnitQuaternion(const UnitQuaternion&) = default;
  UnitQuaternion& operator=(const UnitQuaternion&) = default;

  double w() const { return w_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

  double Get(Axis4d axis) const;

  UnitQuaternion operator*(const UnitQuaternion& other) const;
  void operator*=(const UnitQuaternion& other) { *this = *this * other; }

  UnitQuaternion operator~() const;

  double Angle(const UnitQuaternion& other) const;

  Vector3d Twist(Epsilon small_angle_epsilon = kDefaultSmallAngleEpsilon) const;

  bool GeometricallyEquals(const UnitQuaternion& other, Epsilon epsilon) const;

 private:
  UnitQuaternion(double w, double x, double y, double z);

  double w_, x_, y_, z_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_UNIT_QUATERNION_H_
