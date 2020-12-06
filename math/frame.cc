#include "robotics-common/math/frame.h"

namespace robotics_common {
namespace math {

Frame::Frame() : transform_to_world_(Transform::Identity()) {}

Frame::Frame(const Frame& parent_frame, const Transform& transform_to_parent)
    : transform_to_world_(
          parent_frame.transform_to_world().Append(transform_to_parent)) {}

Frame::Frame(const UnitQuaternion& orientation_in_world,
             const Vector3d& origin_in_world)
    : transform_to_world_(orientation_in_world, origin_in_world) {}

Frame Frame::ApplyTransform(const Transform& transform) const {
  return Frame(*this, transform);
}

}  // namespace math
}  // namespace robotics_common
