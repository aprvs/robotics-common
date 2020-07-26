#ifndef ROBOTICS_COMMON_ROBOT_RIGID_BODY_H_
#define ROBOTICS_COMMON_ROBOT_RIGID_BODY_H_

#include <optional>

#include "robotics-common/math/frame_node.h"

namespace robotics_common {
namespace robot {

class RigidBody {
 public:
  struct InertialParameters {
    double mass;
    double Ixx;
    double Iyy;
    double Izz;
    double Ixy;
    double Iyz;
    double Izx;
  };

  RigidBody(const std::string& name);
  RigidBody(const std::string& name,
            const InertialParameters& inertial_parameters);

  bool HasInertialParameters() { return inertial_parameters_.has_value(); }

  const std::string& name() const { return name_; }

 private:
  const std::string name_;
  std::optional<InertialParameters> inertial_parameters_;
  // TODO(apoorv) add parameters for shape objects and other link properties
};

}  // namespace robot
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_ROBOT_RIGID_BODY_H_
