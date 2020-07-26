#ifndef ROBOTICS_COMMON_MATH_FRAME_MANAGER_H_
#define ROBOTICS_COMMON_MATH_FRAME_MANAGER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/error_or.h"
#include "robotics-common/math/frame_node.h"

namespace robotics_common {
namespace math {

/// @class FrameManager
/// FrameManager is the memory management model for Frame objects
/// Each FrameManager represents a Frame tree rooted at the WorldFrame.
class FrameManager {
 public:
  using FrameSet = std::unordered_set<FrameNode*>;

  FrameManager() = default;

  // FrameManager is not copy constructible
  FrameManager(const FrameManager&) = delete;
  // FrameManager is move constructible
  FrameManager(FrameManager&&) = default;
  // FrameManager is not copy / move constructible / assignable
  FrameManager& operator=(const FrameManager&) = delete;

  const FrameSet& root_frames() const { return root_frames_; }

  common::ErrorOr<FrameNode const*> FindFrame(const std::string& name) const;
  common::ErrorOr<FrameNode*> FindFrame(const std::string& name);

  common::ErrorOr<FrameNode*> CreateFrameRelativeToWorld(
      const std::string& frame_name,
      TransformProvider const* transformProvider);

  common::ErrorOr<FrameNode*> CreateFrameRelativeToWorld(
      const std::string& frame_name);

  common::ErrorOr<FrameNode*> CreateFrame(
      const std::string& frame_name, const std::string& parent_name,
      TransformProvider const* transformProvider);

  common::ErrorOr<FrameNode*> CreateFrame(const std::string& frame_name,
                                          const std::string& parent_name);

 private:
  using FrameMap = std::unordered_map<std::string, FrameNode*>;

  std::vector<std::unique_ptr<FrameNode>> memory_;
  FrameMap frame_map_;
  FrameSet root_frames_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_FRAME_MANAGER_H_
