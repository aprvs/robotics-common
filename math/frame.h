#ifndef ROBOTICS_COMMON_MATH_FRAME_H_
#define ROBOTICS_COMMON_MATH_FRAME_H_

#include "robotics-common/math/transform.h"
#include "robotics-common/math/unit_quaternion.h"
#include "robotics-common/math/vector3d.h"

namespace robotics_common {
namespace math {

class Frame {
 public:
  Frame();
  Frame(const Frame& parent_frame, const Transform& transform_to_parent);
  Frame(const UnitQuaternion& orientation_in_world,
        const Vector3d& origin_in_world);

  // Frame is copy / move constructible
  Frame(const Frame& other) = default;
  // Frame is copy / move assignable
  Frame& operator=(const Frame&) = default;

  const Vector3d& origin_in_world() const {
    return transform_to_world_.linear();
  }

  void set_origin(const Vector3d& origin) {
    transform_to_world_.set_linear(origin);
  }

  const UnitQuaternion& orientation_in_world() const {
    return transform_to_world_.angular();
  }

  void set_orientation(const UnitQuaternion& orientation) {
    transform_to_world_.set_angular(orientation);
  }

  const Transform& transform_to_world() const { return transform_to_world_; }

  Frame ApplyTransform(const Transform& tansform) const;

 private:
  Transform transform_to_world_;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_FRAME_H_
