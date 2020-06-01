#include "robotics-common/math/angle_helper.h"

#include <cmath>

namespace robotics_common {
namespace math {

namespace {

constexpr double kTwoPi = 2 * M_PI;

double WrapAngle(double angle, double range_min) {
  double offset_angle = angle - range_min;
  int32_t quotient = std::floor(offset_angle / kTwoPi);
  double reconstructed = quotient * kTwoPi;
  return angle - reconstructed;
}

}  // namespace

double WrapToPlusMinusPi(double angle) { return WrapAngle(angle, -M_PI); }

double WrapToTwoPi(double angle) { return WrapAngle(angle, 0.0); }

double WrapToNPi(double angle, int32_t half_n) {
  return WrapAngle(angle, half_n * kTwoPi);
}

}  // namespace math
}  // namespace robotics_common
