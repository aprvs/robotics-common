#ifndef ROBOTICS_COMMON_MATH_GEOMETRY_DEFS_H_
#define ROBOTICS_COMMON_MATH_GEOMETRY_DEFS_H_

#include <array>
#include <cstdint>

#include "common/enum_traits.h"

namespace robotics_common {
namespace math {

enum class Axis3d {
  kX,
  kY,
  kZ,
};

Axis3d NextCounterClockwiseAxis(Axis3d axis);
Axis3d NextClockwiseAxis(Axis3d axis);

enum class Axis4d {
  kW,
  kX,
  kY,
  kZ,
};

}  // namespace math
}  // namespace robotics_common

namespace common {

template <>
struct EnumTrait<robotics_common::math::Axis3d> {
  constexpr static size_t num_values() { return 3; }
  constexpr static robotics_common::math::Axis3d default_value() {
    return robotics_common::math::Axis3d::kX;
  }
  constexpr static char const* to_string(robotics_common::math::Axis3d axis) {
    switch (axis) {
      case robotics_common::math::Axis3d::kX:
        return "X";
      case robotics_common::math::Axis3d::kY:
        return "Y";
      case robotics_common::math::Axis3d::kZ:
        return "Z";
        // no default. Catch missing enums at compile time
    }
    return "Unknown";
  }
};

template <>
struct EnumListTrait<robotics_common::math::Axis3d> {
  constexpr static std::array<
      robotics_common::math::Axis3d,
      EnumTrait<robotics_common::math::Axis3d>::num_values()>
  values() {
    return {{robotics_common::math::Axis3d::kX,
             robotics_common::math::Axis3d::kY,
             robotics_common::math::Axis3d::kZ}};
  }
};

template <>
struct EnumTrait<robotics_common::math::Axis4d> {
  constexpr static size_t num_values() { return 4; }
  constexpr static robotics_common::math::Axis4d default_value() {
    return robotics_common::math::Axis4d::kW;
  }
  constexpr static char const* to_string(robotics_common::math::Axis4d axis) {
    switch (axis) {
      case robotics_common::math::Axis4d::kW:
        return "W";
      case robotics_common::math::Axis4d::kX:
        return "X";
      case robotics_common::math::Axis4d::kY:
        return "Y";
      case robotics_common::math::Axis4d::kZ:
        return "Z";
        // no default. Catch missing enums at compile time
    }
    return "Unknown";
  }
};

template <>
struct EnumListTrait<robotics_common::math::Axis4d> {
  constexpr static std::array<
      robotics_common::math::Axis4d,
      EnumTrait<robotics_common::math::Axis4d>::num_values()>
  values() {
    return {
        {robotics_common::math::Axis4d::kW, robotics_common::math::Axis4d::kX,
         robotics_common::math::Axis4d::kY, robotics_common::math::Axis4d::kZ}};
  }
};

}  // namespace common

#endif  // ROBOTICS_COMMON_MATH_GEOMETRY_DEFS_H_
