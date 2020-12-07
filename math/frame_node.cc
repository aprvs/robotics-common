#include "robotics-common/math/frame_node.h"

#include <algorithm>

namespace robotics_common {
namespace math {

FrameNode::FrameNode(const std::string& name, FrameNode* parent_node,
                     TransformProvider const* transform_to_parent_provider)
    : name_(name),
      parent_node_(parent_node),
      transform_to_parent_provider_(transform_to_parent_provider) {
  if (parent_node_ != nullptr) {
    parent_node_->RegisterChildNode(this);
  }
}

bool FrameNode::HasChild(FrameNode const* frame_query) const {
  return std::any_of(child_nodes_.cbegin(), child_nodes_.cend(),
                     [&frame_query](FrameNode const* child_frame) {
                       if (frame_query == child_frame) {
                         return true;
                       }
                       return child_frame->HasChild(frame_query);
                     });
}

bool FrameNode::RegisterChildNode(FrameNode* child_node) {
  if (child_node == nullptr) {
    return false;
  }
  return child_nodes_.insert(child_node).second;
}

}  // namespace math
}  // namespace robotics_common
