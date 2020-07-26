#include "robotics-common/math/frame_manager.h"

#include "gtest/gtest.h"

namespace robotics_common {
namespace math {

TEST(FrameManagerTest, ConstructDestructTest) { FrameManager frame_manager; }

TEST(FrameManagerTest, ConstructFrameTest) {
  FrameManager frame_manager;
  common::ErrorOr<FrameNode*> result =
      frame_manager.CreateFrame("Frame1", "Frame0");
  ASSERT_FALSE(result.HasValue());

  result = frame_manager.CreateFrame("Frame0", "WorldFrame");
  ASSERT_TRUE(result.HasValue());

  FrameNode* frame0 = result.ValueOrDie();
  EXPECT_EQ(frame0->parent_node(), FrameNode::WorldFrame());
  result = frame_manager.CreateFrame("Frame1", "Frame0");
  ASSERT_TRUE(result.HasValue());

  FrameNode* frame1 = result.ValueOrDie();
  EXPECT_EQ(frame1->parent_node(), frame0);
  EXPECT_TRUE(frame0->HasChild(frame1->name()));
  EXPECT_TRUE(frame0->HasChild(frame1));

  result = frame_manager.CreateFrame("Frame1", "WorldFrame");
  EXPECT_FALSE(result.HasValue());
}

}  // namespace math
}  // namespace robotics_common
