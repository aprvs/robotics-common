#include "robotics-common/math/unit_quaternion.h"

#include <cmath>
#include "gtest/gtest.h"

#include "robotics-common/math/unit_vector3d.h"

namespace robotics_common {
namespace math {

TEST(UnitQuaternionTest, ConstructDestructTest) {
  common::ErrorOr<UnitQuaternion> result =
      UnitQuaternion::Construct(0.1, 0.2, 0.3, 0.4, Epsilon::kOneBillionth);
  EXPECT_FALSE(result.HasValue());

  result = UnitQuaternion::Construct(0.428, 0.719, 0.208, -0.507,
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
  UnitQuaternion q1 =
      UnitQuaternion::Construct(0.5, -0.5, -0.5, -0.5, Epsilon::kOneBillionth)
          .ValueOrDie();
  UnitQuaternion q2 = ~q1;
  EXPECT_DOUBLE_EQ(q2.w(), 0.5);
  EXPECT_DOUBLE_EQ(q2.x(), 0.5);
  EXPECT_DOUBLE_EQ(q2.y(), 0.5);
  EXPECT_DOUBLE_EQ(q2.z(), 0.5);
}

TEST(UnitQuaternionTest, MultiplicationTest) {
  UnitQuaternion q1 =
      UnitQuaternion::Construct(0.5, -0.5, -0.5, -0.5, Epsilon::kOneBillionth)
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
  common::ErrorOr<UnitQuaternion> result = UnitQuaternion::Construct(
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
  common::ErrorOr<UnitQuaternion> result = UnitQuaternion::Construct(
      kSqtHalf, kSqtHalf, 0.0, 0.0, Epsilon::kOneBillionth);
  ASSERT_TRUE(result.HasValue());
  UnitQuaternion q1 = result.ValueOrDie();
  EXPECT_DOUBLE_EQ(q1.Angle(UnitQuaternion::kIdentity), M_PI / 2.0);

  UnitQuaternion q2 = q1 * q1;
  EXPECT_DOUBLE_EQ(q2.Angle(UnitQuaternion::kIdentity), M_PI);
}

TEST(UnitQuaternionTest, AxisAngleTest) {
  UnitQuaternion q =
      UnitQuaternion::FromAxisAngle(M_PI / 3, UnitVector3d::kXAxis);
  EXPECT_DOUBLE_EQ(q.w(), std::sqrt(0.75));
  EXPECT_DOUBLE_EQ(q.x(), std::sqrt(0.25));
  EXPECT_DOUBLE_EQ(q.y(), 0.0);
  EXPECT_DOUBLE_EQ(q.z(), 0.0);

  q = UnitQuaternion::FromAxisAngle(
      M_PI / 3,
      UnitVector3d::Construct(0.5, 0.5, 0.5, Epsilon::kOne).ValueOrDie());
  EXPECT_DOUBLE_EQ(q.w(), std::sqrt(0.75));
  EXPECT_DOUBLE_EQ(q.x(), std::sqrt(1.0 / 12.0));
  EXPECT_DOUBLE_EQ(q.y(), std::sqrt(1.0 / 12.0));
  EXPECT_DOUBLE_EQ(q.z(), std::sqrt(1.0 / 12.0));
}

TEST(UnitQuaternionTest, FromTwistTest) {
  UnitQuaternion q = UnitQuaternion::FromTwist(Vector3d(0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(q.w(), 1.0);
  EXPECT_DOUBLE_EQ(q.x(), 0.0);
  EXPECT_DOUBLE_EQ(q.y(), 0.0);
  EXPECT_DOUBLE_EQ(q.z(), 0.0);

  q = UnitQuaternion::FromTwist(M_PI / 4.0 * Vector3d(1.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(q.w(), std::cos(M_PI / 8.0));
  EXPECT_DOUBLE_EQ(q.x(), std::sin(M_PI / 8.0));
  EXPECT_DOUBLE_EQ(q.y(), 0.0);
  EXPECT_DOUBLE_EQ(q.z(), 0.0);

  q = UnitQuaternion::FromTwist(M_PI / 4.0 * Vector3d(0.0, -1.0, 0.0));
  EXPECT_DOUBLE_EQ(q.w(), std::cos(M_PI / 8.0));
  EXPECT_DOUBLE_EQ(q.x(), 0.0);
  EXPECT_DOUBLE_EQ(q.y(), -std::sin(M_PI / 8.0));
  EXPECT_DOUBLE_EQ(q.z(), 0.0);

  q = UnitQuaternion::FromTwist(Vector3d(1e-20, 1e-17, 1e-8));
  EXPECT_DOUBLE_EQ(q.w(), 1.0);
  EXPECT_DOUBLE_EQ(q.x(), 5e-21);
  EXPECT_DOUBLE_EQ(q.y(), 5e-18);
  EXPECT_DOUBLE_EQ(q.z(), 5e-9);
  // check to make sure that the type safety is maintained
  double norm = q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
  EXPECT_EQ(norm, 1.0);
}

TEST(UnitQuaternionTest, ToTwistTest) {
  Vector3d twist = UnitQuaternion::kIdentity.Twist();
  EXPECT_DOUBLE_EQ(twist.x(), 0.0);
  EXPECT_DOUBLE_EQ(twist.y(), 0.0);
  EXPECT_DOUBLE_EQ(twist.z(), 0.0);

  twist = UnitQuaternion::FromAxisAngle(M_PI, UnitVector3d::kXAxis).Twist();
  EXPECT_DOUBLE_EQ(twist.x(), M_PI);
  EXPECT_DOUBLE_EQ(twist.y(), 0.0);
  EXPECT_DOUBLE_EQ(twist.z(), 0.0);

  twist = UnitQuaternion::FromAxisAngle(-M_PI, UnitVector3d::kXAxis).Twist();
  EXPECT_DOUBLE_EQ(twist.x(), -M_PI);
  EXPECT_DOUBLE_EQ(twist.y(), 0.0);
  EXPECT_DOUBLE_EQ(twist.z(), 0.0);

  twist =
      UnitQuaternion::FromAxisAngle(-1.1 * M_PI, UnitVector3d::kXAxis).Twist();
  EXPECT_DOUBLE_EQ(twist.x(), -0.9 * M_PI);
  EXPECT_DOUBLE_EQ(twist.y(), 0.0);
  EXPECT_DOUBLE_EQ(twist.z(), 0.0);

  twist =
      UnitQuaternion::FromAxisAngle(
          -M_PI,
          UnitVector3d::Construct(1.0, 1.0, 1.0, Epsilon::kOne).ValueOrDie())
          .Twist();
  EXPECT_DOUBLE_EQ(twist.x(), -M_PI / std::sqrt(3.0));
  EXPECT_DOUBLE_EQ(twist.y(), -M_PI / std::sqrt(3.0));
  EXPECT_DOUBLE_EQ(twist.z(), -M_PI / std::sqrt(3.0));
}

TEST(UnitQuaternionTest, GeometricallyEqualsTest) {
  EXPECT_TRUE(UnitQuaternion::FromAxisAngle(M_PI, UnitVector3d::kXAxis)
                  .GeometricallyEquals(UnitQuaternion::FromAxisAngle(
                                           -M_PI, UnitVector3d::kXAxis),
                                       Epsilon::kTenMillionth));
}

}  // namespace math
}  // namespace robotics_common
