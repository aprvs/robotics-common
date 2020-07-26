#include "robotics-common/math/frame_manager.h"

namespace robotics_common {
namespace math {

common::ErrorOr<FrameNode const*> FrameManager::FindFrame(
    const std::string& name) const {
  FrameMap::const_iterator it = frame_map_.find(name);
  if (it == frame_map_.cend()) {
    return common::Error::kNotFound;
  }
  return it->second;
}

common::ErrorOr<FrameNode*> FrameManager::FindFrame(const std::string& name) {
  FrameMap::iterator it = frame_map_.find(name);
  if (it == frame_map_.cend()) {
    return common::Error::kNotFound;
  }
  return it->second;
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrameRelativeToWorld(
    const std::string& frame_name, TransformProvider const* transformProvider) {
  return CreateFrame(frame_name, FrameNode::WorldFrame()->name(),
                     transformProvider);
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrameRelativeToWorld(
    const std::string& frame_name) {
  return CreateFrameRelativeToWorld(frame_name, nullptr);
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrame(
    const std::string& frame_name, const std::string& parent_name) {
  return CreateFrame(frame_name, parent_name, nullptr);
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrame(
    const std::string& frame_name, const std::string& parent_name,
    TransformProvider const* transformProvider) {
  if (FindFrame(frame_name).HasValue()) {
    return common::Error::kInvalidArgument;
  }
  FrameNode const* parent_frame_ptr = FrameNode::WorldFrame();
  FrameNode* mutable_parent_ptr = nullptr;

  if (parent_name != parent_frame_ptr->name()) {
    auto parent_search_result = FindFrame(parent_name);
    if (parent_search_result.HasError()) {
      return common::Error::kInvalidArgument;
    }
    parent_frame_ptr = parent_search_result.ValueOrDie();
    mutable_parent_ptr = parent_search_result.ValueOrDie();
  }

  std::unique_ptr<FrameNode> new_frame = std::unique_ptr<FrameNode>(
      new FrameNode(frame_name, parent_frame_ptr, transformProvider));
  FrameNode* new_frame_ptr = new_frame.get();
  memory_.emplace_back(std::move(new_frame));

  frame_map_.insert({new_frame_ptr->name(), new_frame_ptr});

  if (mutable_parent_ptr != nullptr) {
    mutable_parent_ptr->RegisterChildNode(new_frame_ptr);
  } else {
    root_frames_.insert(new_frame_ptr);
  }

  return new_frame_ptr;
}

}  // namespace math
}  // namespace robotics_common
