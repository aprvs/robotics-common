#ifndef ROBOTICS_COMMON_MATH_FRAME_H_
#define ROBOTICS_COMMON_MATH_FRAME_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/error_or.h"

namespace robotics_common {
namespace math {

/// @class FrameManager
/// FrameManager is the memory management model for Frame objects
/// Each FrameManager represents a Frame tree rooted at the WorldFrame.
class FrameManager {
 public:
  /// @class Frame
  /// Frame is a representation of physical frames, a set of three mutually
  /// perpendicular axes in SE(3). Logically, frames are nodes of a tree rooted
  /// in an arbitrary WorldFrame. This allows for frame characteristics/traits
  /// to be inherited and psuedo-automated generation of transforms between
  /// frames. Since Frame objects must reference each other (possibly in a
  /// cyclic fashion) the FrameManager class provides for memory management
  class Frame {
   public:
    static Frame const* WorldFrame();

    // Frames are not copy / move constructible / assignable
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;

    const std::string& name() const { return name_; }
    Frame const* parent_frame() const { return parent_frame_; }
    FrameManager const* manager() const { return manager_; }

   private:
    friend class FrameManager;
    Frame(const std::string& name, Frame const* parent_frame,
          FrameManager const* manager);

    const std::string name_;
    Frame const* parent_frame_;
    FrameManager const* manager_;
    std::vector<Frame const*> derived_frames_;
  };

  FrameManager() = default;

  // FrameManager is not copy / move constructible / assignable
  FrameManager(const FrameManager&) = delete;
  FrameManager& operator=(const FrameManager&) = delete;

  common::ErrorOr<Frame const*> GetFrame(const std::string& frame_name) const;
  common::ErrorOr<Frame*> GetFrame(const std::string& frame_name);

  common::ErrorOr<Frame*> CreateFrameRelativeToWorld(
      const std::string& frame_name);
  common::ErrorOr<Frame*> CreateFrame(const std::string& frame_name,
                                      const std::string& parent_name);

 private:
  using FrameMap = std::unordered_map<std::string, Frame*>;

  std::vector<std::unique_ptr<Frame>> memory_;
  FrameMap frame_map_;
};

using FrameNode = FrameManager::Frame;

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_FRAME_H_
