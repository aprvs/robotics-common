#include "robotics-common/math/angle_helper.h"

#include <cmath>

#include "gtest/gtest.h"
#include "robotics-common/math/epsilons.h"

namespace robotics_common {
namespace math {

TEST(AngleHelperTest, WrapToPlusMinusPi) {
  EXPECT_DOUBLE_EQ(WrapToPlusMinusPi(0.75 * M_PI), 0.75 * M_PI);
  EXPECT_DOUBLE_EQ(WrapToPlusMinusPi(3 * M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(WrapToPlusMinusPi(-3 * M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(WrapToPlusMinusPi(-1.5 * M_PI), 0.5 * M_PI);
  // Loss in numerical precision with these functions aware from zero
  EXPECT_NEAR(WrapToPlusMinusPi(-10.5 * M_PI), -0.5 * M_PI,
              EpsilonValue(Epsilon::kOneBillionth));
}

TEST(AngleHelperTest, WrapToTwoPiTest) {
  EXPECT_DOUBLE_EQ(WrapToTwoPi(0.75 * M_PI), 0.75 * M_PI);
  EXPECT_DOUBLE_EQ(WrapToTwoPi(3 * M_PI), M_PI);
  EXPECT_DOUBLE_EQ(WrapToTwoPi(-3 * M_PI), M_PI);
  EXPECT_DOUBLE_EQ(WrapToTwoPi(-1.5 * M_PI), 0.5 * M_PI);
  // Loss in numerical precision with these functions aware from zero
  EXPECT_NEAR(WrapToTwoPi(-10.5 * M_PI), 1.5 * M_PI,
              EpsilonValue(Epsilon::kOneBillionth));
}

TEST(AngleHelperTest, WrapToNPiTest) {
  EXPECT_DOUBLE_EQ(WrapToNPi(0.75 * M_PI, 1), 2.75 * M_PI);
  EXPECT_DOUBLE_EQ(WrapToNPi(3.0 * M_PI, 1), 3 * M_PI);
  EXPECT_DOUBLE_EQ(WrapToNPi(-3.0 * M_PI, -2), -3.0 * M_PI);
  EXPECT_DOUBLE_EQ(WrapToNPi(-3.0 * M_PI, 4), 9 * M_PI);
  EXPECT_DOUBLE_EQ(WrapToNPi(-1.5 * M_PI, 2), 4.5 * M_PI);
  // Loss in numerical precision with these functions aware from zero
  EXPECT_NEAR(WrapToNPi(-10.5 * M_PI, -6), -10.5 * M_PI,
              EpsilonValue(Epsilon::kOneBillionth));
}

}  // namespace math
}  // namespace robotics_common
