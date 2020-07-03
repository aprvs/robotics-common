#include "robotics-common/math/frame_manager.h"

namespace robotics_common {
namespace math {

// static
FrameNode const* FrameManager::Frame::WorldFrame() {
  static const Frame world_frame("WorldFrame", nullptr, nullptr);
  return &world_frame;
}

FrameManager::Frame::Frame(const std::string& name, Frame const* parent_frame,
                           FrameManager const* manager)
    : name_(name), parent_frame_(parent_frame), manager_(manager) {}

common::ErrorOr<FrameNode const*> FrameManager::GetFrame(
    const std::string& frame_name) const {
  FrameMap::const_iterator it = frame_map_.find(frame_name);
  if (it == frame_map_.cend()) {
    return common::Error::kNotFound;
  }
  return it->second;
}

common::ErrorOr<FrameNode*> FrameManager::GetFrame(
    const std::string& frame_name) {
  FrameMap::iterator it = frame_map_.find(frame_name);
  if (it == frame_map_.cend()) {
    return common::Error::kNotFound;
  }
  return it->second;
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrameRelativeToWorld(
    const std::string& frame_name) {
  return CreateFrame(frame_name, Frame::WorldFrame()->name());
}

common::ErrorOr<FrameNode*> FrameManager::CreateFrame(
    const std::string& frame_name, const std::string& parent_name) {
  if (GetFrame(frame_name).HasValue()) {
    return common::Error::kInvalidArgument;
  }
  Frame const* parent_frame_ptr = Frame::WorldFrame();
  if (parent_name != parent_frame_ptr->name()) {
    auto parent_search_result = GetFrame(parent_name);
    if (parent_search_result.HasError()) {
      return common::Error::kInvalidArgument;
    }
    parent_frame_ptr = parent_search_result.ValueOrDie();
  }
  memory_.emplace_back(
      std::unique_ptr<Frame>(new Frame(frame_name, parent_frame_ptr, this)));
  Frame* new_frame = memory_.rbegin()->get();
  frame_map_.insert({new_frame->name(), new_frame});
  return new_frame;
}

}  // namespace math
}  // namespace robotics_common
