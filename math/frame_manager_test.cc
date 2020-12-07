#include "robotics-common/math/frame_manager.h"

#include <gtest/gtest.h>

namespace robotics_common {
namespace math {

TEST(FrameManagerTest, ConstructDestructTest) { FrameManager frame_manager; }

TEST(FrameManagerTest, CreateFrameTest) {
  FrameManager frame_manager;

  // Attempt frame creation from an unregistered frame
  common::ErrorOr<FrameNode*> result =
      frame_manager.CreateFrame("Frame1", "Frame0");
  ASSERT_FALSE(result.HasValue());

  // Create frame relative to world
  result = frame_manager.CreateFrameRelativeToWorld("Frame0");
  ASSERT_TRUE(result.HasValue());
  FrameNode* frame0 = result.ValueOrDie();
  EXPECT_EQ(frame0->parent_node(), nullptr);
  EXPECT_EQ(frame0->name(), "Frame0");

  // Create frame relative to a registered frame
  result = frame_manager.CreateFrame("Frame1", "Frame0");
  ASSERT_TRUE(result.HasValue());
  FrameNode* frame1 = result.ValueOrDie();
  EXPECT_EQ(frame1->parent_node(), frame0);
  EXPECT_TRUE(frame0->HasChild(frame1));
  EXPECT_EQ(frame1->name(), "Frame1");

  // Attempt re-creating a registered frame
  result = frame_manager.CreateFrameRelativeToWorld("Frame1");
  EXPECT_FALSE(result.HasValue());

  // Create frame relative to world
  result = frame_manager.CreateFrame("Frame2", "WorldFrame");
  ASSERT_TRUE(result.HasValue());
  FrameNode* frame2 = result.ValueOrDie();
  EXPECT_EQ(frame2->parent_node(), nullptr);
  EXPECT_EQ(frame2->name(), "Frame2");
}

TEST(FrameManagerTest, CreateInvalidFrameTest) {
  // Validate creating of frames with restricted names
  FrameManager frame_manager;
  common::ErrorOr<FrameNode*> result =
      frame_manager.CreateFrameRelativeToWorld("WorldFrame");
  ASSERT_FALSE(result.HasValue());

  result = frame_manager.CreateFrameRelativeToWorld("Frame0");
  ASSERT_TRUE(result.HasValue());

  result = frame_manager.CreateFrame("WorldFrame", "Frame0");
  ASSERT_FALSE(result.HasValue());
}

TEST(FrameManagerTest, CreateFrameAndValidateAssociationsTest) {
  FrameManager frame_manager;

  auto result = frame_manager.CreateFrame("TestFrame1", "WorldFrame");
  ASSERT_TRUE(result.HasValue());

  FrameNode* frame1 = result.ValueOrDie();
  EXPECT_EQ(frame1->parent_node(), nullptr);
  EXPECT_FALSE(frame1->has_parent_node());
  EXPECT_EQ(frame1->transform_to_parent_provider(), nullptr);
  EXPECT_EQ(frame1->child_nodes().size(), 0);

  result = frame_manager.CreateFrame("TestFrame2", "TestFrame1");
  ASSERT_TRUE(result.HasValue());

  FrameNode* frame2 = result.ValueOrDie();
  EXPECT_EQ(frame2->parent_node(), frame1);
  EXPECT_TRUE(frame2->has_parent_node());
  EXPECT_EQ(frame2->transform_to_parent_provider(), nullptr);
  EXPECT_EQ(frame2->child_nodes().size(), 0);
  EXPECT_EQ(frame1->child_nodes().size(), 1);
  EXPECT_TRUE(frame1->child_nodes().find(frame2) !=
              frame1->child_nodes().cend());

  result = frame_manager.CreateFrame("TestFrame3", "TestFrame1");
  ASSERT_TRUE(result.HasValue());

  FrameNode* frame3 = result.ValueOrDie();
  EXPECT_EQ(frame3->parent_node(), frame1);
  EXPECT_TRUE(frame3->has_parent_node());
  EXPECT_EQ(frame3->transform_to_parent_provider(), nullptr);
  EXPECT_EQ(frame1->child_nodes().size(), 2);
  EXPECT_EQ(frame2->child_nodes().size(), 0);
  EXPECT_EQ(frame3->child_nodes().size(), 0);
  EXPECT_TRUE(frame1->child_nodes().find(frame2) !=
              frame1->child_nodes().cend());
  EXPECT_TRUE(frame1->child_nodes().find(frame3) !=
              frame1->child_nodes().cend());

  result = frame_manager.CreateFrame("TestFrame4", "TestFrame2");
  ASSERT_TRUE(result.HasValue());

  FrameNode* frame4 = result.ValueOrDie();
  EXPECT_EQ(frame4->parent_node(), frame2);
  EXPECT_TRUE(frame4->has_parent_node());
  EXPECT_EQ(frame4->transform_to_parent_provider(), nullptr);
  EXPECT_EQ(frame1->child_nodes().size(), 2);
  EXPECT_EQ(frame2->child_nodes().size(), 1);
  EXPECT_EQ(frame3->child_nodes().size(), 0);
  EXPECT_EQ(frame4->child_nodes().size(), 0);
  EXPECT_TRUE(frame1->child_nodes().find(frame2) !=
              frame1->child_nodes().cend());
  EXPECT_TRUE(frame1->child_nodes().find(frame3) !=
              frame1->child_nodes().cend());
  EXPECT_TRUE(frame1->child_nodes().find(frame4) ==
              frame1->child_nodes().cend());
  EXPECT_TRUE(frame2->child_nodes().find(frame4) !=
              frame2->child_nodes().cend());
}

}  // namespace math
}  // namespace robotics_common
