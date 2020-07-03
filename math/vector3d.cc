#include "robotics-common/math/vector3d.h"

namespace robotics_common {
namespace math {

Vector3d::Vector3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}

// static
const Vector3d& Vector3d::Zero() {
  static const Vector3d kZero(0.0, 0.0, 0.0);
  return kZero;
}

void Vector3d::Set(Axis3d axis, double value) {
  switch (axis) {
    case Axis3d::kX:
      set_x(value);
      return;
    case Axis3d::kY:
      set_y(value);
      return;
    case Axis3d::kZ:
      set_z(value);
      return;
      // no default. Catch missing cases at compile time
  }
}

}  // namespace math
}  // namespace robotics_common
