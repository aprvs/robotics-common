#ifndef ROBOTICS_COMMON_ROBOT_JOINT_H_
#define ROBOTICS_COMMON_ROBOT_JOINT_H_

#include <string>
#include <vector>

#include "robotics-common/math/transform_provider.h"
#include "robotics-common/math/unit_quaternion.h"
#include "robotics-common/math/vector3d.h"

namespace robotics_common {
namespace robot {

class Joint : public math::TransformProvider {
 public:
  Joint(const std::string& name);
  ~Joint() override = default;

  virtual int32_t NumDofs() const = 0;

  const std::string& name() const { return name_; }

 private:
  const std::string name_;
};

class MultiAxisJoint : public Joint {
 public:
  MultiAxisJoint(const std::string& name);
  ~MultiAxisJoint() override;
};

class SixDofJoint : public MultiAxisJoint {
 public:
  SixDofJoint(const std::string& name);
  ~SixDofJoint() override;

  int32_t NumDofs() const final;

  math::Transform FromParentTransform() const final;
  math::Transform ToParentTransform() const final;

  math::UnitQuaternion OrientationInParentFrame() const final;
  math::Vector3d OriginInParentFrame() const final;

  math::Vector3d AngularVelocityInParentFrame() const final;
  math::Vector3d LinearVelocityInParentFrame() const final;

  math::Vector3d AngularAccelerationInParentFrame() const final;
  math::Vector3d LinearAccelerationInParentFrame() const final;
};

class OneDofJoint : public Joint {
 public:
  struct State {
    double q;
    double dq;
    double ddq;
  };

  OneDofJoint(const std::string& name, math::Axis3d axis);
  ~OneDofJoint() override;

  int32_t NumDofs() const final { return 1; }

  void SetState(const State& state);

 private:
  State state_;
  math::Axis3d axis_;
};

class PrismaticJoint : public OneDofJoint {
 public:
  PrismaticJoint(const std::string& name, math::Axis3d axis);
  ~PrismaticJoint() override;

  math::Transform FromParentTransform() const final;
  math::Transform ToParentTransform() const final;

  math::UnitQuaternion OrientationInParentFrame() const final;
  math::Vector3d OriginInParentFrame() const final;

  math::Vector3d AngularVelocityInParentFrame() const final;
  math::Vector3d LinearVelocityInParentFrame() const final;

  math::Vector3d AngularAccelerationInParentFrame() const final;
  math::Vector3d LinearAccelerationInParentFrame() const final;

 private:
};

class RevoluteJoint : public OneDofJoint {
 public:
  RevoluteJoint(const std::string& name, math::Axis3d axis);
  ~RevoluteJoint() override;

  math::Transform FromParentTransform() const final;
  math::Transform ToParentTransform() const final;

  math::UnitQuaternion OrientationInParentFrame() const final;
  math::Vector3d OriginInParentFrame() const final;

  math::Vector3d AngularVelocityInParentFrame() const final;
  math::Vector3d LinearVelocityInParentFrame() const final;

  math::Vector3d AngularAccelerationInParentFrame() const final;
  math::Vector3d LinearAccelerationInParentFrame() const final;

 private:
};

class FixedJoint : public Joint {
 public:
  FixedJoint(const std::string& name);
  ~FixedJoint() override;

  int32_t NumDofs() const final;

  math::Transform FromParentTransform() const final;
  math::Transform ToParentTransform() const final;

  math::UnitQuaternion OrientationInParentFrame() const final;
  math::Vector3d OriginInParentFrame() const final;

  math::Vector3d AngularVelocityInParentFrame() const final;
  math::Vector3d LinearVelocityInParentFrame() const final;

  math::Vector3d AngularAccelerationInParentFrame() const final;
  math::Vector3d LinearAccelerationInParentFrame() const final;
};

class Constraint : public Joint {
 public:
  Constraint(const std::string& name);
  ~Constraint() override;

  int32_t NumDofs() const final;

  math::Transform FromParentTransform() const final;
  math::Transform ToParentTransform() const final;

  math::UnitQuaternion OrientationInParentFrame() const final;
  math::Vector3d OriginInParentFrame() const final;

  math::Vector3d AngularVelocityInParentFrame() const final;
  math::Vector3d LinearVelocityInParentFrame() const final;

  math::Vector3d AngularAccelerationInParentFrame() const final;
  math::Vector3d LinearAccelerationInParentFrame() const final;
};

}  // namespace robot
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_ROBOT_JOINT_H_
