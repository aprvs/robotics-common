#include "robotics-common/math/frame.h"

#include <cmath>

#include <gtest/gtest.h>

namespace robotics_common {
namespace math {

namespace {

const double kHalf = 0.5;
const double kSquareRootHalf = std::sqrt(kHalf);
const double kSquareRootThreeQuarters = kHalf * std::sqrt(3.0);

}  // namespace

TEST(FrameTest, ConstructDestructTest) {
  Frame identity;
  Frame from_world(UnitQuaternion::Identity(), Vector3d::Zero());
  Frame from_parent(from_world,
                    Transform(UnitQuaternion::Identity(), Vector3d::Zero()));
}

TEST(FrameTest, ConstructionFromValueTest) {
  Frame f(UnitQuaternion::FromTwist(Vector3d(0.5 * M_PI, 0.0, 0.0)),
          Vector3d(0.5, 0.4, 0.2));

  EXPECT_DOUBLE_EQ(f.origin_in_world().x(), 0.5);
  EXPECT_DOUBLE_EQ(f.origin_in_world().y(), 0.4);
  EXPECT_DOUBLE_EQ(f.origin_in_world().z(), 0.2);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().w(), kSquareRootHalf);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().x(), kSquareRootHalf);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().y(), 0.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().z(), 0.0);
}

TEST(FrameTest, ConstructionFromParentTest) {
  Frame frame1(UnitQuaternion::FromTwist(Vector3d(0.5 * M_PI, 0.0, 0.0)),
               Vector3d(0.5, 0.4, 0.2));
  Frame frame2(frame1,
               Transform(UnitQuaternion::FromTwist({0.0, 0.25 * M_PI, 0.0}),
                         {1.0, 2.0, 3.0}));

  constexpr double kEpsilon = 3e-5;
  EXPECT_DOUBLE_EQ(frame2.origin_in_world().x(), 1.5);
  EXPECT_DOUBLE_EQ(frame2.origin_in_world().y(), -2.6);
  EXPECT_DOUBLE_EQ(frame2.origin_in_world().z(), 2.2);
  EXPECT_NEAR(frame2.orientation_in_world().w(), 0.6532899, kEpsilon);
  EXPECT_NEAR(frame2.orientation_in_world().x(), 0.6532899, kEpsilon);
  EXPECT_NEAR(frame2.orientation_in_world().y(), 0.2705776, kEpsilon);
  EXPECT_NEAR(frame2.orientation_in_world().z(), 0.2705776, kEpsilon);
}

TEST(FrameTest, ApplyTransformTest) {
  Frame frame1(UnitQuaternion::FromTwist(Vector3d(0.0, 0.0, -M_PI / 3.0)),
               Vector3d(10.0, 5.0, -5.0));

  Frame frame2 = frame1.ApplyTransform(Transform::Identity());
  EXPECT_DOUBLE_EQ(frame2.origin_in_world().x(), 10.0);
  EXPECT_DOUBLE_EQ(frame2.origin_in_world().y(), 5.0);
  EXPECT_DOUBLE_EQ(frame2.origin_in_world().z(), -5.0);
  EXPECT_DOUBLE_EQ(frame2.orientation_in_world().w(), kSquareRootThreeQuarters);
  EXPECT_DOUBLE_EQ(frame2.orientation_in_world().x(), 0.0);
  EXPECT_DOUBLE_EQ(frame2.orientation_in_world().y(), 0.0);
  EXPECT_DOUBLE_EQ(frame2.orientation_in_world().z(), -kHalf);
}

TEST(FrameTest, SetterGetterTest) {
  Frame f;
  f.set_origin(Vector3d(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(f.origin_in_world().x(), 1.0);
  EXPECT_DOUBLE_EQ(f.origin_in_world().y(), 2.0);
  EXPECT_DOUBLE_EQ(f.origin_in_world().z(), 3.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().w(), 1.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().x(), 0.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().y(), 0.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().z(), 0.0);

  f.set_orientation(UnitQuaternion::FromTwist(Vector3d(0.0, M_PI, 0.0)));
  EXPECT_DOUBLE_EQ(f.origin_in_world().x(), 1.0);
  EXPECT_DOUBLE_EQ(f.origin_in_world().y(), 2.0);
  EXPECT_DOUBLE_EQ(f.origin_in_world().z(), 3.0);
  EXPECT_NEAR(f.orientation_in_world().w(), 0.0, 1e-10);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().x(), 0.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().y(), 1.0);
  EXPECT_DOUBLE_EQ(f.orientation_in_world().z(), 0.0);
}

}  // namespace math
}  // namespace robotics_common
