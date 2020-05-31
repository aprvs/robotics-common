#include "robotics-common/math/unit_quaternion.h"

#include <cmath>

namespace robotics_common {
namespace math {

// static
const UnitQuaternion UnitQuaternion::kIdentity =
    UnitQuaternion(1.0, 0.0, 0.0, 0.0);

common::ErrorOr<UnitQuaternion> UnitQuaternion::Construct(double w, double x,
                                                          double y, double z,
                                                          Epsilon epsilon) {
  double length = std::sqrt(w * w + x * x + y * y + z * z);
  if (std::fabs(length - 1.0) > EpsilonValue(epsilon)) {
    return common::Error::kInvalidArgument;
  }
  return UnitQuaternion(w / length, x / length, y / length, z / length);
}

UnitQuaternion::UnitQuaternion(double w, double x, double y, double z)
    : w_(w), x_(x), y_(y), z_(z) {}

double UnitQuaternion::Get(Axis4d axis) const {
  switch (axis) {
    case Axis4d::kW:
      return w();
    case Axis4d::kX:
      return x();
    case Axis4d::kY:
      return y();
    case Axis4d::kZ:
      return z();
      // no default. Catch missing enum cases at compile time
  }
  return 0.0;
}

UnitQuaternion UnitQuaternion::operator*(const UnitQuaternion& other) const {
  double res_w =
      w_ * other.w() - x_ * other.x() - y_ * other.y() - z_ * other.z();
  double res_x =
      w_ * other.x() + x_ * other.w() + y_ * other.z() - z_ * other.y();
  double res_y =
      w_ * other.y() - x_ * other.z() + y_ * other.w() + z_ * other.x();
  double res_z =
      w_ * other.z() + x_ * other.y() - y_ * other.x() + z_ * other.w();
  return UnitQuaternion(res_w, res_x, res_y, res_z);
}

UnitQuaternion UnitQuaternion::operator~() const {
  return UnitQuaternion(w_, -x_, -y_, -z_);
}

double UnitQuaternion::Angle(const UnitQuaternion& other) const {
  return 2.0 * std::acos(w_ * other.w() + x_ * other.x() + y_ * other.y() +
                         z_ * other.z());
}

}  // namespace math
}  // namespace robotics_common
