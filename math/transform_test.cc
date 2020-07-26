#include "robotics-common/math/transform.h"

#include "gtest/gtest.h"

namespace robotics_common {
namespace math {

TEST(TransformTest, ConstructDestructTest) {
  Transform t = Transform::Identity();
  EXPECT_DOUBLE_EQ(t.linear().x(), 0.0);
  EXPECT_DOUBLE_EQ(t.linear().x(), 0.0);
  EXPECT_DOUBLE_EQ(t.linear().x(), 0.0);

  EXPECT_DOUBLE_EQ(t.angular().w(), 1.0);
  EXPECT_DOUBLE_EQ(t.angular().x(), 0.0);
  EXPECT_DOUBLE_EQ(t.angular().y(), 0.0);
  EXPECT_DOUBLE_EQ(t.angular().z(), 0.0);
}

TEST(TransformTest, CompositionTest) {
  const Vector3d l(2.0, 3.0, 4.0);
  const UnitQuaternion q =
      UnitQuaternion::Construct(0.5, 0.5, 0.5, 0.5, Epsilon::kOneBillionth)
          .ValueOrDie();
  Transform t1 = Transform(UnitQuaternion::Identity(), l);
  Transform t2 = Transform(q, Vector3d::Zero());

  Transform appended = t1.Append(t2);
  EXPECT_DOUBLE_EQ(appended.linear().x(), 2.0);
  EXPECT_DOUBLE_EQ(appended.linear().y(), 3.0);
  EXPECT_DOUBLE_EQ(appended.linear().z(), 4.0);

  EXPECT_DOUBLE_EQ(appended.angular().w(), 0.5);
  EXPECT_DOUBLE_EQ(appended.angular().x(), 0.5);
  EXPECT_DOUBLE_EQ(appended.angular().y(), 0.5);
  EXPECT_DOUBLE_EQ(appended.angular().z(), 0.5);

  Transform prepend = t2.Prepend(t1);
  EXPECT_DOUBLE_EQ(prepend.linear().x(), 2.0);
  EXPECT_DOUBLE_EQ(prepend.linear().y(), 3.0);
  EXPECT_DOUBLE_EQ(prepend.linear().z(), 4.0);

  EXPECT_DOUBLE_EQ(prepend.angular().w(), 0.5);
  EXPECT_DOUBLE_EQ(prepend.angular().x(), 0.5);
  EXPECT_DOUBLE_EQ(prepend.angular().y(), 0.5);
  EXPECT_DOUBLE_EQ(prepend.angular().z(), 0.5);

  EXPECT_TRUE(appended.GeometricallyEquals(prepend, Epsilon::kOneBillionth,
                                           Epsilon::kOneBillionth));
}

TEST(TransformTest, MathTest) {
  const Vector3d l(2.0, 3.0, 4.0);
  const UnitQuaternion q =
      UnitQuaternion::Construct(0.5, 0.5, 0.5, 0.5, Epsilon::kOneBillionth)
          .ValueOrDie();
  Transform t = Transform(q, l);
  Transform inv_t = ~t;

  Transform i = t.Prepend(inv_t);
  EXPECT_DOUBLE_EQ(i.linear().x(), 0.0);
  EXPECT_DOUBLE_EQ(i.linear().y(), 0.0);
  EXPECT_DOUBLE_EQ(i.linear().z(), 0.0);

  EXPECT_DOUBLE_EQ(i.angular().w(), 1.0);
  EXPECT_DOUBLE_EQ(i.angular().x(), 0.0);
  EXPECT_DOUBLE_EQ(i.angular().y(), 0.0);
  EXPECT_DOUBLE_EQ(i.angular().z(), 0.0);
}

}  // namespace math
}  // namespace robotics_common
