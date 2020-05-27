#include "robotics-common/math/epsilons.h"

namespace robotics_common {
namespace math {

double EpsilonValue(Epsilon epsilon) {
  switch (epsilon) {
    case Epsilon::kOne:
      return 1.0e0;
    case Epsilon::kOneTenth:
      return 1.0e-1;
    case Epsilon::kOneHundredth:
      return 1.0e-2;
    case Epsilon::kOneThousandth:
      return 1.0e-3;
    case Epsilon::kOneTenThousandth:
      return 1.0e-4;
    case Epsilon::kOneHundredThousandth:
      return 1.0e-5;
    case Epsilon::kOneMillionth:
      return 1.0e-6;
    case Epsilon::kTenMillionth:
      return 1.0e-7;
    case Epsilon::kHundredMillionth:
      return 1.0e-8;
    case Epsilon::kBillionth:
      return 1.0e-9;
      // no default. Catch missing enum cases at compile time
  }
  return 1.0e0;
}

}  // namespace math
}  // namespace robotics_common
