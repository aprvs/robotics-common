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
  EXPECT_EQ(frame0->parent_frame(), FrameNode::WorldFrame());
  result = frame_manager.CreateFrame("Frame1", "Frame0");
  ASSERT_TRUE(result.HasValue());
  FrameNode* frame1 = result.ValueOrDie();
  EXPECT_EQ(frame1->parent_frame(), frame0);
  result = frame_manager.CreateFrame("Frame1", "WorldFrame");
  EXPECT_FALSE(result.HasValue());
}

}  // namespace math
}  // namespace robotics_common
