#ifndef ROBOTICS_COMMON_MATH_UNIT_QUATERNION_H_
#define ROBOTICS_COMMON_MATH_UNIT_QUATERNION_H_

#include "common/error_or.h"
#include "robotics-common/math/epsilons.h"
#include "robotics-common/math/geometry_defs.h"

namespace robotics_common {
namespace math {

class UnitQuaternion {
 public:
  static const UnitQuaternion kIdentity;
  static common::ErrorOr<UnitQuaternion> NormalizeAndCreate(double w, double x,
                                                            double y, double z,
                                                            Epsilon epsilon);

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

 private:
  UnitQuaternion(double w, double x, double y, double z);

  double w_, x_, y_, z_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_UNIT_QUATERNION_H_