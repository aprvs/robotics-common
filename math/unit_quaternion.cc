#include "robotics-common/math/unit_quaternion.h"

#include "robotics-common/math/angle_helper.h"
#include "robotics-common/math/unit_vector3d.h"

#include <cmath>

namespace robotics_common {
namespace math {

// static
const UnitQuaternion UnitQuaternion::kIdentity =
    UnitQuaternion(1.0, 0.0, 0.0, 0.0);

// static
const UnitQuaternion& UnitQuaternion::Identity() {
  return UnitQuaternion::kIdentity;
}

// static
common::ErrorOr<UnitQuaternion> UnitQuaternion::Construct(double w, double x,
                                                          double y, double z,
                                                          Epsilon epsilon) {
  double length = std::sqrt(w * w + x * x + y * y + z * z);
  if (std::fabs(length - 1.0) >= EpsilonValue(epsilon)) {
    return common::Error::kInvalidArgument;
  }
  return UnitQuaternion(w / length, x / length, y / length, z / length);
}

// static
UnitQuaternion UnitQuaternion::FromAxisAngle(double angle,
                                             const UnitVector3d& axis) {
  double half_angle = 0.5 * angle;
  double sine = std::sin(half_angle);
  return UnitQuaternion(std::cos(half_angle), sine * axis.x(), sine * axis.y(),
                        sine * axis.z());
}

// static
UnitQuaternion UnitQuaternion::FromTwist(const Vector3d& axis,
                                         Epsilon small_angle_epsilon) {
  double length = axis.Length();
  if (length < EpsilonValue(small_angle_epsilon)) {
    // small angle approximation
    double cos = std::sqrt(1.0 - 0.25 * length * length);
    return UnitQuaternion(cos, 0.5 * axis.x(), 0.5 * axis.y(), 0.5 * axis.z());
  }
  double half_angle = 0.5 * length;
  double scale = std::sin(half_angle) / length;
  return UnitQuaternion(std::cos(half_angle), scale * axis.x(),
                        scale * axis.y(), scale * axis.z());
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

Vector3d UnitQuaternion::Twist(Epsilon small_angle_epsilon) const {
  double length = std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
  if (length < EpsilonValue(small_angle_epsilon)) {
    return Vector3d(x_, y_, z_);
  }
  double angle = Angle(UnitQuaternion::kIdentity);
  if (w_ > 0.0) {
    return (angle / length) * Vector3d(x_, y_, z_);
  }
  return ((angle - 2 * M_PI) / length) * Vector3d(-x_, -y_, -z_);
}

bool UnitQuaternion::GeometricallyEquals(const UnitQuaternion& other,
                                         Epsilon epsilon) const {
  double angle = Angle(other);
  return WrapToPlusMinusPi(angle) < EpsilonValue(epsilon);
}

}  // namespace math
}  // namespace robotics_common
