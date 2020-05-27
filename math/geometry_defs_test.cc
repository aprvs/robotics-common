#include "robotics-common/math/geometry_defs.h"

#include "gtest/gtest.h"

namespace robotics_common {
namespace math {

TEST(GeometryDefsTest, AxisOrderingTest) {
  EXPECT_EQ(NextCounterClockwiseAxis(Axis3d::kX), Axis3d::kY);
  EXPECT_EQ(NextCounterClockwiseAxis(Axis3d::kY), Axis3d::kZ);
  EXPECT_EQ(NextCounterClockwiseAxis(Axis3d::kZ), Axis3d::kX);

  EXPECT_EQ(NextClockwiseAxis(Axis3d::kX), Axis3d::kZ);
  EXPECT_EQ(NextClockwiseAxis(Axis3d::kY), Axis3d::kX);
  EXPECT_EQ(NextClockwiseAxis(Axis3d::kZ), Axis3d::kY);
}

}  // namespace math
}  // namespace robotics_common
