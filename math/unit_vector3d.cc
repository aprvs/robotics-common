#include "robotics-common/math/unit_vector3d.h"

#include <algorithm>
#include <cmath>

namespace robotics_common {
namespace math {

const UnitVector3d UnitVector3d::kXAxis(1.0, 0.0, 0.0);
const UnitVector3d UnitVector3d::kYAxis(0.0, 1.0, 0.0);
const UnitVector3d UnitVector3d::kZAxis(0.0, 0.0, 1.0);

const UnitVector3d UnitVector3d::kAxes[3] = {kXAxis, kYAxis, kZAxis};

UnitVector3d::UnitVector3d() : UnitVector3d(1.0, 0.0, 0.0) {}

UnitVector3d::UnitVector3d(double x, double y, double z)
    : x_(x), y_(y), z_(z) {}

common::ErrorOr<UnitVector3d> UnitVector3d::Construct(double x, double y,
                                                      double z,
                                                      Epsilon epsilon) {
  double length = std::sqrt(x * x + y * y + z * z);
  if (length < EpsilonValue(epsilon)) {
    return common::Error::kInvalidArgument;
  }
  return UnitVector3d(x / length, y / length, z / length);
}

double UnitVector3d::Angle(const UnitVector3d& other) const {
  double dot_product = std::max(-1.0, std::min(1.0, Dot(other)));
  return std::acos(dot_product);
}

UnitVector3d UnitVector3d::Cross(const UnitVector3d& other) const {
  return Cross<UnitVector3d>(other);
}

void UnitVector3d::Negate() { *this = -*this; }

void UnitVector3d::Set(Axis3d axis, double value) {
  switch (axis) {
    case Axis3d::kX:
      x_ = value;
      return;
    case Axis3d::kY:
      y_ = value;
      return;
    case Axis3d::kZ:
      z_ = value;
      return;
      // no default. Catch missing enums cases at compile time
  }
}

}  // namespace math
}  // namespace robotics_common
