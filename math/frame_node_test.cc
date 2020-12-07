#include "robotics-common/math/frame_node.h"

#include "robotics-common/math/transform_provider.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace robotics_common {
namespace math {

class MockTransform final : public TransformProvider {
 public:
  ~MockTransform() override = default;

  MOCK_METHOD(Transform, FromParentTransform, (), (const, override));
  MOCK_METHOD(Transform, ToParentTransform, (), (const, override));

  MOCK_METHOD(UnitQuaternion, OrientationInParentFrame, (), (const, override));
  MOCK_METHOD(Vector3d, OriginInParentFrame, (), (const, override));

  MOCK_METHOD(Vector3d, AngularVelocityInParentFrame, (), (const, override));
  MOCK_METHOD(Vector3d, LinearVelocityInParentFrame, (), (const, override));

  MOCK_METHOD(Vector3d, AngularAccelerationInParentFrame, (),
              (const, override));
  MOCK_METHOD(Vector3d, LinearAccelerationInParentFrame, (), (const, override));
};

TEST(FrameNodeTest, ConstructDesctructTest) {
  FrameNode node{"TestFrame", nullptr, nullptr};
  EXPECT_EQ(node.name(), "TestFrame");
  EXPECT_EQ(node.parent_node(), nullptr);
  EXPECT_FALSE(node.has_parent_node());
  EXPECT_EQ(node.transform_to_parent_provider(), nullptr);
  EXPECT_EQ(node.child_nodes().size(), 0);

  MockTransform transform1;
  FrameNode parent_node("Node1", nullptr, &transform1);
  EXPECT_EQ(parent_node.name(), "Node1");
  EXPECT_EQ(parent_node.parent_node(), nullptr);
  EXPECT_FALSE(parent_node.has_parent_node());
  EXPECT_EQ(parent_node.transform_to_parent_provider(), &transform1);
  EXPECT_EQ(parent_node.child_nodes().size(), 0);

  MockTransform transform2;
  FrameNode child_node("Node2", &parent_node, &transform2);
  EXPECT_EQ(child_node.name(), "Node2");
  EXPECT_EQ(child_node.parent_node(), &parent_node);
  EXPECT_TRUE(child_node.has_parent_node());
  EXPECT_EQ(child_node.transform_to_parent_provider(), &transform2);
  EXPECT_EQ(child_node.child_nodes().size(), 0);
  ASSERT_EQ(parent_node.child_nodes().size(), 1);
  EXPECT_TRUE(parent_node.child_nodes().find(&child_node) !=
              parent_node.child_nodes().end());
}

}  // namespace math
}  // namespace robotics_common
