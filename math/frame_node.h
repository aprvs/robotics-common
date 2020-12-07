#ifndef ROBOTICS_COMMON_MATH_FRAME_NODE_H_
#define ROBOTICS_COMMON_MATH_FRAME_NODE_H_

#include <string>
#include <unordered_set>

#include "robotics-common/math/transform_provider.h"

namespace robotics_common {
namespace math {

class FrameNode {
 public:
  using FramePtrSet = std::unordered_set<FrameNode const*>;

  FrameNode(const std::string& name, FrameNode* parent_node,
            TransformProvider const* transform_to_parent_provider);

  // FrameNode is not copy / move constructible / assignable
  FrameNode(const FrameNode&) = delete;
  FrameNode& operator=(const FrameNode&) = delete;

  const std::string& name() const { return name_; }

  FrameNode const* parent_node() const { return parent_node_; }
  bool has_parent_node() const { return parent_node() != nullptr; }

  TransformProvider const* transform_to_parent_provider() const {
    return transform_to_parent_provider_;
  }

  const FramePtrSet& child_nodes() const { return child_nodes_; }

  bool HasChild(FrameNode const* frame_query) const;

 private:
  bool RegisterChildNode(FrameNode* child_node);

  /// Name / label of the frame
  const std::string name_;
  /// Pointer to the FrameNode relative to which this is defined
  /// Parent node pointers should be null for ::WorldFrame(). All other frame
  /// nodes have valid non-null parent node pointers.
  FrameNode* parent_node_;
  /// Pointer to object that provides transforms to the FrameNode::parent_node_
  TransformProvider const* transform_to_parent_provider_;
  /// List of pointers to FrameNode objects that are defined relative to this
  FramePtrSet child_nodes_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_FRAME_NODE_H_
