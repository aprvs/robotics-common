#ifndef ROBOTICS_COMMON_MATH_HELPER_H_
#define ROBOTICS_COMMON_MATH_HELPER_H_

#include <cstdint>

namespace robotics_common {
namespace math {

double WrapToPlusMinusPi(double angle);

double WrapToTwoPi(double angle);

double WrapToNPi(double angle, int32_t half_n);

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_HELPER_H_
