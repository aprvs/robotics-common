#include "robotics-common/math/transform.h"

namespace robotics_common {
namespace math {

Transform::Transform(const UnitQuaternion& angular, const Vector3d& linear)
    : angular_(angular), linear_(linear) {}

// static
const Transform& Transform::Identity() {
  static const Transform kIdentity(UnitQuaternion::Identity(),
                                   Vector3d::Zero());
  return kIdentity;
}

Transform Transform::Prepend(const Transform& pre) const {
  return Transform(pre.angular() * angular(),
                   linear().Rotate(pre.angular()) + pre.linear());
}

Transform Transform::Append(const Transform& post) const {
  return Transform(angular() * post.angular(),
                   post.linear().Rotate(angular()) + linear());
}

Transform Transform::operator~() const {
  const auto inverted_angular = ~angular();
  return Transform(inverted_angular, -linear().Rotate(inverted_angular));
}

bool Transform::GeometricallyEquals(const Transform& other,
                                    Epsilon angular_epsilon,
                                    Epsilon linear_epsilon) {
  return linear_.GeometricallyEquals(other.linear(), linear_epsilon) &&
         angular_.GeometricallyEquals(other.angular(), angular_epsilon);
}

}  // namespace math
}  // namespace robotics_common
