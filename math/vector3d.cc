#include "robotics-common/math/vector3d.h"

namespace robotics_common {
namespace math {

Vector3d::Vector3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}

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
