#include "robotics-common/math/frame_manager.h"

#include <algorithm>
#include <string>

namespace robotics_common {
namespace math {

namespace {

const std::string kWorldFrameName = "WorldFrame";
const std::string kForbiddenNames[] = {kWorldFrameName};

bool IsValidFrameName(const std::string& name_to_validate) {
  return std::none_of(std::begin(kForbiddenNames), std::end(kForbiddenNames),
                      [&name_to_validate](const std::string& invalid_name) {
                        return invalid_name == name_to_validate;
                      });
}

}  // namespace

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
  if (it == frame_map_.end()) {
    return common::Error::kNotFound;
  }
  return it->second;
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrameRelativeToWorld(
    const std::string& frame_name,
    TransformProvider const* transform_provider) {
  if (FindFrame(frame_name).HasValue() || !IsValidFrameName(frame_name)) {
    return common::Error::kInvalidArgument;
  }
  return CreateAndRegisterFrame(frame_name, nullptr, transform_provider);
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrame(
    const std::string& frame_name, const std::string& parent_name,
    TransformProvider const* transform_provider) {
  if (FindFrame(frame_name).HasValue() || !IsValidFrameName(frame_name)) {
    return common::Error::kInvalidArgument;
  }

  FrameNode* parent = nullptr;
  if (parent_name != kWorldFrameName) {
    auto parent_search_result = FindFrame(parent_name);
    if (parent_search_result.HasError()) {
      return common::Error::kInvalidArgument;
    }
    parent = parent_search_result.ValueOrDie();
  }

  return CreateAndRegisterFrame(frame_name, parent, transform_provider);
}

FrameNode* FrameManager::CreateAndRegisterFrame(
    const std::string& name, FrameNode* parent_node,
    TransformProvider const* transform_provider) {
  // Create the frame
  std::unique_ptr<FrameNode> frame =
      std::make_unique<FrameNode>(name, parent_node, transform_provider);

  // Add to internal data structures
  FrameNode* frame_ptr = frame.get();
  frame_map_.insert({name, frame_ptr});

  // Add to parent frame
  if (parent_node == nullptr) {
    root_frames_.push_back(frame_ptr);
  }

  // Move to memory cache
  memory_.push_back(std::move(frame));
  return frame_ptr;
}

}  // namespace math
}  // namespace robotics_common
