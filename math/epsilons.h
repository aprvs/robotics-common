#ifndef ROBOTICS_COMMON_MATH_EPSILONS_H_
#define ROBOTICS_COMMON_MATH_EPSILONS_H_

#include "common/enum_traits.h"

namespace robotics_common {
namespace math {

// Following the definitions from
// http://academics.wellesley.edu/Astronomy/kmcleod/Toolkit/scinot.html
/// @class Epsilon
enum class Epsilon {
  kOne,
  kOneTenth,
  kOneHundredth,
  kOneThousandth,
  kOneTenThousandth,
  kOneHundredThousandth,
  kOneMillionth,
  kTenMillionth,
  kHundredMillionth,
  kOneBillionth,
};

/// @param[in] epsilon  epsilon enumeration
/// @return  floating point value corresponding to the precision
double EpsilonValue(Epsilon epsilon);

}  // namespace math
}  // namespace robotics_common

namespace common {

template <>
struct EnumTrait<robotics_common::math::Epsilon> {
  constexpr static size_t num_values() { return 10; }
  constexpr static char const* to_string(
      robotics_common::math::Epsilon epsilon) {
    switch (epsilon) {
      case robotics_common::math::Epsilon::kOne:
        return "1.0";
      case robotics_common::math::Epsilon::kOneTenth:
        return "1.0e-1";
      case robotics_common::math::Epsilon::kOneHundredth:
        return "1.0e-2";
      case robotics_common::math::Epsilon::kOneThousandth:
        return "1.0e-3";
      case robotics_common::math::Epsilon::kOneTenThousandth:
        return "1.0e-4";
      case robotics_common::math::Epsilon::kOneHundredThousandth:
        return "1.0e-5";
      case robotics_common::math::Epsilon::kOneMillionth:
        return "1.0e-6";
      case robotics_common::math::Epsilon::kTenMillionth:
        return "1.0e-7";
      case robotics_common::math::Epsilon::kHundredMillionth:
        return "1.0e-8";
      case robotics_common::math::Epsilon::kOneBillionth:
        return "1.0e-9";
        // no default. Catch missing enum cases at compile time
    }
    return "Unknown";
  }
};

}  // namespace common

#endif  // ROBOTICS_COMMON_MATH_EPSILONS_H_
