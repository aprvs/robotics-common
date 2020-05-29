#include "robotics-common/math/unit_quaternion.h"

#include <cmath>
#include "gtest/gtest.h"

namespace robotics_common {
namespace math {

TEST(UnitQuaternionTest, ConstructDestructTest) {
  common::ErrorOr<UnitQuaternion> result = UnitQuaternion::NormalizeAndCreate(
      0.1, 0.2, 0.3, 0.4, Epsilon::kOneBillionth);
  EXPECT_FALSE(result.HasValue());

  result = UnitQuaternion::NormalizeAndCreate(0.428, 0.719, 0.208, -0.507,
                                              Epsilon::kOneThousandth);
  ASSERT_TRUE(result.HasValue());
  UnitQuaternion q1 = result.ValueOrDie();
  EXPECT_DOUBLE_EQ(q1.w(), 0.42790202165427749);
  EXPECT_DOUBLE_EQ(q1.x(), 0.71883540553604097);
  EXPECT_DOUBLE_EQ(q1.y(), 0.20795238435534982);
  EXPECT_DOUBLE_EQ(q1.z(), -0.50688393686616517);

  UnitQuaternion q2(q1);
  EXPECT_DOUBLE_EQ(q2.w(), 0.42790202165427749);
  EXPECT_DOUBLE_EQ(q2.x(), 0.71883540553604097);
  EXPECT_DOUBLE_EQ(q2.y(), 0.20795238435534982);
  EXPECT_DOUBLE_EQ(q2.z(), -0.50688393686616517);
}

TEST(UnitQuaternionTest, InverseTest) {
  UnitQuaternion q1 = UnitQuaternion::NormalizeAndCreate(0.5, -0.5, -0.5, -0.5,
                                                         Epsilon::kOneBillionth)
                          .ValueOrDie();
  UnitQuaternion q2 = ~q1;
  EXPECT_DOUBLE_EQ(q2.w(), 0.5);
  EXPECT_DOUBLE_EQ(q2.x(), 0.5);
  EXPECT_DOUBLE_EQ(q2.y(), 0.5);
  EXPECT_DOUBLE_EQ(q2.z(), 0.5);
}

TEST(UnitQuaternionTest, MultiplicationTest) {
  UnitQuaternion q1 = UnitQuaternion::NormalizeAndCreate(0.5, -0.5, -0.5, -0.5,
                                                         Epsilon::kOneBillionth)
                          .ValueOrDie();
  UnitQuaternion q2 = q1 * UnitQuaternion::kIdentity;
  EXPECT_DOUBLE_EQ(q2.w(), 0.5);
  EXPECT_DOUBLE_EQ(q2.x(), -0.5);
  EXPECT_DOUBLE_EQ(q2.y(), -0.5);
  EXPECT_DOUBLE_EQ(q2.z(), -0.5);

  UnitQuaternion q3 = q1 * ~q1;
  EXPECT_DOUBLE_EQ(q3.w(), 1.0);
  EXPECT_DOUBLE_EQ(q3.x(), 0.0);
  EXPECT_DOUBLE_EQ(q3.y(), 0.0);
  EXPECT_DOUBLE_EQ(q3.z(), 0.0);

  q3 = ~q1 * q1;
  EXPECT_DOUBLE_EQ(q3.w(), 1.0);
  EXPECT_DOUBLE_EQ(q3.x(), 0.0);
  EXPECT_DOUBLE_EQ(q3.y(), 0.0);
  EXPECT_DOUBLE_EQ(q3.z(), 0.0);

  UnitQuaternion q4 = q1 * q1;
  EXPECT_DOUBLE_EQ(q4.w(), -0.5);
  EXPECT_DOUBLE_EQ(q4.x(), -0.5);
  EXPECT_DOUBLE_EQ(q4.y(), -0.5);
  EXPECT_DOUBLE_EQ(q4.z(), -0.5);

  const double kSqtHalf = 1.0 / std::sqrt(2.0);
  common::ErrorOr<UnitQuaternion> result = UnitQuaternion::NormalizeAndCreate(
      kSqtHalf, kSqtHalf, 0.0, 0.0, Epsilon::kOneBillionth);
  ASSERT_TRUE(result.HasValue());
  UnitQuaternion q5 = result.ValueOrDie();
  UnitQuaternion q6 = q5 * q5;
  EXPECT_DOUBLE_EQ(q6.w(), 0.0);
  EXPECT_DOUBLE_EQ(q6.x(), 1.0);
  EXPECT_DOUBLE_EQ(q6.y(), 0.0);
  EXPECT_DOUBLE_EQ(q6.z(), 0.0);
}

TEST(UnitQuaternionTest, AngleTest) {
  const double kSqtHalf = 1.0 / std::sqrt(2.0);
  common::ErrorOr<UnitQuaternion> result = UnitQuaternion::NormalizeAndCreate(
      kSqtHalf, kSqtHalf, 0.0, 0.0, Epsilon::kOneBillionth);
  ASSERT_TRUE(result.HasValue());
  UnitQuaternion q1 = result.ValueOrDie();
  EXPECT_DOUBLE_EQ(q1.Angle(UnitQuaternion::kIdentity), M_PI / 2.0);

  UnitQuaternion q2 = q1 * q1;
  EXPECT_DOUBLE_EQ(q2.Angle(UnitQuaternion::kIdentity), M_PI);
}

}  // namespace math

}  // namespace robotics_common
