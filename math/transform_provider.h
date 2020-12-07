#ifndef ROBOTICS_COMMON_MATH_TRANSFORM_PROVIDER_H_
#define ROBOTICS_COMMON_MATH_TRANSFORM_PROVIDER_H_

#include "robotics-common/math/transform.h"
#include "robotics-common/math/unit_quaternion.h"
#include "robotics-common/math/vector3d.h"

namespace robotics_common {
namespace math {

class TransformProvider {
 public:
  virtual ~TransformProvider() = default;

  virtual Transform FromParentTransform() const = 0;
  virtual Transform ToParentTransform() const = 0;

  virtual UnitQuaternion OrientationInParentFrame() const = 0;
  virtual Vector3d OriginInParentFrame() const = 0;

  virtual Vector3d AngularVelocityInParentFrame() const = 0;
  virtual Vector3d LinearVelocityInParentFrame() const = 0;

  virtual Vector3d AngularAccelerationInParentFrame() const = 0;
  virtual Vector3d LinearAccelerationInParentFrame() const = 0;
};

}  // namespace math
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_MATH_TRANSFORM_PROVIDER_H_
