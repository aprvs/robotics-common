#include "robotics-common/math/vector3d.h"

#include "gtest/gtest.h"

namespace robotics_common {
namespace math {

TEST(Vector3dTest, ConstructDestructTest) {
  Vector3d vector1(1.0, 2.0, 3.0);
  Vector3d vector2;
  Vector3d vector3(3.0, 2.0);

  Vector3d vector4(vector1);
  vector2 = vector3;
}

TEST(Vector3dTest, GetterTest) {
  Vector3d vector(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(vector.x(), 1.0);
  EXPECT_DOUBLE_EQ(vector.y(), 2.0);
  EXPECT_DOUBLE_EQ(vector.z(), 3.0);
}

TEST(Vector3dTest, SetterTest) {
  Vector3d vector(1.0, 2.0, 3.0);
  vector.set_x(2.0);
  vector.set_y(10.1);
  vector.set_z(-1.0);
  EXPECT_DOUBLE_EQ(vector.x(), 2.0);
  EXPECT_DOUBLE_EQ(vector.y(), 10.1);
  EXPECT_DOUBLE_EQ(vector.z(), -1.0);
}

TEST(Vector3dTest, ScalingTest) {
  Vector3d vector(1.0, 2.0, 3.0);
  vector *= 3.0;
  EXPECT_DOUBLE_EQ(vector.x(), 3.0);
  EXPECT_DOUBLE_EQ(vector.y(), 6.0);
  EXPECT_DOUBLE_EQ(vector.z(), 9.0);
  Vector3d scaled_vector = vector * 3.14;
  EXPECT_DOUBLE_EQ(scaled_vector.x(), 9.42);
  EXPECT_DOUBLE_EQ(scaled_vector.y(), 18.84);
  EXPECT_DOUBLE_EQ(scaled_vector.z(), 28.26);

  scaled_vector = -2.0 * scaled_vector;
  EXPECT_DOUBLE_EQ(scaled_vector.x(), -18.84);
  EXPECT_DOUBLE_EQ(scaled_vector.y(), -37.68);
  EXPECT_DOUBLE_EQ(scaled_vector.z(), -56.52);
}

TEST(Vector3dTest, AdditionTest) {
  Vector3d vector = Vector3d(1.0, 0.0, 0.0) + Vector3d(0.0, 1.0, 0.0) +
                    Vector3d(0.0, 0.0, 2.0);
  EXPECT_DOUBLE_EQ(vector.x(), 1.0);
  EXPECT_DOUBLE_EQ(vector.y(), 1.0);
  EXPECT_DOUBLE_EQ(vector.z(), 2.0);

  vector += Vector3d(-12.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(vector.x(), -11.0);
  EXPECT_DOUBLE_EQ(vector.y(), 1.0);
  EXPECT_DOUBLE_EQ(vector.z(), 2.0);
}

TEST(Vector3dTest, SubtractionTest) {
  Vector3d vector = Vector3d(1.0, 0.0, 0.0) - Vector3d(0.0, 1.0, 0.0) -
                    Vector3d(0.0, 0.0, -2.0);
  EXPECT_DOUBLE_EQ(vector.x(), 1.0);
  EXPECT_DOUBLE_EQ(vector.y(), -1.0);
  EXPECT_DOUBLE_EQ(vector.z(), 2.0);

  vector -= Vector3d(-12.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(vector.x(), 13.0);
  EXPECT_DOUBLE_EQ(vector.y(), -1.0);
  EXPECT_DOUBLE_EQ(vector.z(), 2.0);
}

TEST(Vector3dTest, DotProductTest) {
  EXPECT_DOUBLE_EQ(Vector3d(1.0, 0.0, 0.0).Dot(Vector3d(3.14, 0.0, 0.0)), 3.14);
  EXPECT_DOUBLE_EQ(Vector3d(1.0, 4.0, 9.0).Dot(Vector3d(-2.0, 4.0, 8.8)), 93.2);
}

TEST(Vector3dTest, CrossProductTest) {
  Vector3d cross_product =
      Vector3d(2.0, 0.0, 0.0).Cross(Vector3d(0.0, 3.0, 0.0));
  EXPECT_DOUBLE_EQ(cross_product.x(), 0.0);
  EXPECT_DOUBLE_EQ(cross_product.y(), 0.0);
  EXPECT_DOUBLE_EQ(cross_product.z(), 6.0);

  cross_product = Vector3d(2.0, 3.0, 0.0).Cross(Vector3d(-1.0, -1.5, 0.0));
  EXPECT_DOUBLE_EQ(cross_product.x(), 0.0);
  EXPECT_DOUBLE_EQ(cross_product.y(), 0.0);
  EXPECT_DOUBLE_EQ(cross_product.z(), 0.0);
}

TEST(Vector3dTest, TripleProductTest) {
  EXPECT_DOUBLE_EQ(
      TripleProduct(Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 2.0),
                    Vector3d(0.0, 1.0, 0.0)),
      -2.0);
}

}  // namespace math
}  // namespace robotics_common
