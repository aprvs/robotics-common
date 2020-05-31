#include "robotics-common/math/unit_vector3d.h"

#include "gtest/gtest.h"

namespace robotics_common {
namespace math {

TEST(UnitVector3dTest, ConstructDestructTest) { UnitVector3d unit_vector; }

TEST(UnitVector3dTest, NormalizeConstructTest) {
  common::ErrorOr<UnitVector3d> result =
      UnitVector3d::Construct(0.0, 0.5, 0.0, Epsilon::kTenMillionth);
  EXPECT_FALSE(result.HasValue());

  result = UnitVector3d::Construct(0.0, 0.0, 0.0, Epsilon::kTenMillionth);
  EXPECT_FALSE(result.HasValue());

  result =
      UnitVector3d::Construct(1.0 + 0.9e-7, 0.0, 0.0, Epsilon::kTenMillionth);
  ASSERT_TRUE(result.HasValue());
  UnitVector3d unit_vector = result.ValueOrDie();
  EXPECT_DOUBLE_EQ(unit_vector.Length(), 1.0);
  EXPECT_DOUBLE_EQ(unit_vector.x(), 1.0);
  EXPECT_DOUBLE_EQ(unit_vector.y(), 0.0);
  EXPECT_DOUBLE_EQ(unit_vector.z(), 0.0);
}

TEST(UnitVector3dTest, CrossProductTest) {
  UnitVector3d unit_vector = UnitVector3d::kXAxis.Cross(UnitVector3d::kYAxis);
  EXPECT_DOUBLE_EQ(unit_vector.x(), 0.0);
  EXPECT_DOUBLE_EQ(unit_vector.y(), 0.0);
  EXPECT_DOUBLE_EQ(unit_vector.z(), 1.0);

  unit_vector = UnitVector3d::kYAxis.Cross(UnitVector3d::kZAxis);
  EXPECT_DOUBLE_EQ(unit_vector.x(), 1.0);
  EXPECT_DOUBLE_EQ(unit_vector.y(), 0.0);
  EXPECT_DOUBLE_EQ(unit_vector.z(), 0.0);

  unit_vector = UnitVector3d::kZAxis.Cross(UnitVector3d::kXAxis);
  EXPECT_DOUBLE_EQ(unit_vector.x(), 0.0);
  EXPECT_DOUBLE_EQ(unit_vector.y(), 1.0);
  EXPECT_DOUBLE_EQ(unit_vector.z(), 0.0);
}

TEST(UnitVector3dTest, NegationTest) {
  UnitVector3d unit_vector = -UnitVector3d::kXAxis;
  EXPECT_DOUBLE_EQ(unit_vector.x(), -1.0);
  EXPECT_DOUBLE_EQ(unit_vector.y(), 0.0);
  EXPECT_DOUBLE_EQ(unit_vector.z(), 0.0);

  unit_vector.Negate();
  EXPECT_TRUE(unit_vector.GeometricallyEquals(UnitVector3d::kXAxis,
                                              Epsilon::kOneBillionth));
}

TEST(UnitVector3dTest, DotProductTest) {
  EXPECT_DOUBLE_EQ(UnitVector3d::kXAxis.Dot(UnitVector3d::kXAxis), 1.0);
  EXPECT_DOUBLE_EQ(UnitVector3d::kYAxis.Dot(UnitVector3d::kYAxis), 1.0);
  EXPECT_DOUBLE_EQ(UnitVector3d::kZAxis.Dot(UnitVector3d::kZAxis), 1.0);

  EXPECT_DOUBLE_EQ(UnitVector3d::kXAxis.Dot(UnitVector3d::kYAxis), 0.0);
  EXPECT_DOUBLE_EQ(UnitVector3d::kXAxis.Dot(UnitVector3d::kZAxis), 0.0);

  EXPECT_DOUBLE_EQ(UnitVector3d::kYAxis.Dot(UnitVector3d::kXAxis), 0.0);
  EXPECT_DOUBLE_EQ(UnitVector3d::kYAxis.Dot(UnitVector3d::kZAxis), 0.0);

  EXPECT_DOUBLE_EQ(UnitVector3d::kZAxis.Dot(UnitVector3d::kXAxis), 0.0);
  EXPECT_DOUBLE_EQ(UnitVector3d::kZAxis.Dot(UnitVector3d::kYAxis), 0.0);
}

TEST(UnitVector3dTest, EqualsCheckTest) {
  EXPECT_TRUE(UnitVector3d::kXAxis.GeometricallyEquals(UnitVector3d::kXAxis,
                                                       Epsilon::kOneBillionth));
  EXPECT_TRUE(UnitVector3d::kYAxis.GeometricallyEquals(UnitVector3d::kYAxis,
                                                       Epsilon::kOneBillionth));
  EXPECT_TRUE(UnitVector3d::kZAxis.GeometricallyEquals(UnitVector3d::kZAxis,
                                                       Epsilon::kOneBillionth));

  EXPECT_FALSE(UnitVector3d::kXAxis.GeometricallyEquals(
      UnitVector3d::kYAxis, Epsilon::kOneBillionth));
  EXPECT_FALSE(UnitVector3d::kYAxis.GeometricallyEquals(
      UnitVector3d::kZAxis, Epsilon::kOneBillionth));
  EXPECT_FALSE(UnitVector3d::kZAxis.GeometricallyEquals(
      UnitVector3d::kXAxis, Epsilon::kOneBillionth));
}

TEST(UnitVector3dTest, ComputeAngleBetweenVectors) {
  UnitVector3d v =
      UnitVector3d::Construct(1, 0, 0, Epsilon::kOneBillionth).ValueOrDie();
  UnitVector3d w = UnitVector3d::Construct(std::sqrt(0.5), std::sqrt(0.5), 0,
                                           Epsilon::kOneBillionth)
                       .ValueOrDie();
  EXPECT_DOUBLE_EQ(v.Angle(w), M_PI * 0.25);
}

TEST(UnitVector3dTest, TripleProduct) {
  double triple_product = TripleProduct(
      UnitVector3d::kXAxis, UnitVector3d::kYAxis, UnitVector3d::kZAxis);
  EXPECT_DOUBLE_EQ(triple_product, 1.0);

  triple_product = TripleProduct(UnitVector3d::kXAxis, UnitVector3d::kXAxis,
                                 UnitVector3d::kXAxis);
  EXPECT_DOUBLE_EQ(triple_product, 0.0);
}

TEST(UnitVector3dTest, RotateTest) {
  UnitQuaternion q =
      UnitQuaternion::Construct(0.0, 1.0, 0.0, 0.0, Epsilon::kOneBillionth)
          .ValueOrDie();
  UnitVector3d result = UnitVector3d::kXAxis.Rotate(q);
  EXPECT_TRUE(
      result.GeometricallyEquals(UnitVector3d::kXAxis, Epsilon::kOneBillionth));

  q = UnitQuaternion::Construct(0.0, 0.0, 1.0, 0.0, Epsilon::kOneBillionth)
          .ValueOrDie();
  result = UnitVector3d::kXAxis.Rotate(q);
  EXPECT_TRUE(result.GeometricallyEquals(-UnitVector3d::kXAxis,
                                         Epsilon::kOneBillionth));

  q = UnitQuaternion::Construct(std::cos(M_PI / 4), 0.0, 0.0,
                                std::sin(M_PI / 4), Epsilon::kOneBillionth)
          .ValueOrDie();
  result = UnitVector3d::kXAxis.Rotate(q);
  EXPECT_TRUE(
      result.GeometricallyEquals(UnitVector3d::kYAxis, Epsilon::kOneBillionth));

  q = UnitQuaternion::Construct(std::cos(M_PI / 12), 0.0, 0.0,
                                std::sin(M_PI / 12), Epsilon::kOneBillionth)
          .ValueOrDie();
  result = UnitVector3d::kXAxis.Rotate(q);
  EXPECT_DOUBLE_EQ(result.x(), std::cos(M_PI / 6));
  EXPECT_DOUBLE_EQ(result.y(), std::sin(M_PI / 6));
  EXPECT_DOUBLE_EQ(result.z(), 0.0);

  q = UnitQuaternion::Construct(0.5, 0.5, 0.5, 0.5, Epsilon::kOneBillionth)
          .ValueOrDie();
  result = UnitVector3d::kXAxis.Rotate(q);
  EXPECT_TRUE(
      result.GeometricallyEquals(UnitVector3d::kYAxis, Epsilon::kOneBillionth));
}

}  // namespace math
}  // namespace robotics_common
