#include "robotics-common/math/geometry_defs.h"

namespace robotics_common {
namespace math {

Axis3d NextCounterClockwiseAxis(Axis3d axis) {
  switch (axis) {
    case Axis3d::kX:
      return Axis3d::kY;
    case Axis3d::kY:
      return Axis3d::kZ;
    case Axis3d::kZ:
      return Axis3d::kX;
      // no default. Catch missing enums at compile time
  }
  return Axis3d::kX;
}

Axis3d NextClockwiseAxis(Axis3d axis) {
  switch (axis) {
    case Axis3d::kX:
      return Axis3d::kZ;
    case Axis3d::kY:
      return Axis3d::kX;
    case Axis3d::kZ:
      return Axis3d::kY;
      // no default. Catch missing enums at compile time
  }
  return Axis3d::kX;
}

}  // namespace math
}  // namespace robotics_common
