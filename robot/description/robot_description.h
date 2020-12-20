#ifndef ROBOTICS_COMMON_ROBOT_DESCRIPITON_ROBOT_DESCRIPTION_H_
#define ROBOTICS_COMMON_ROBOT_DESCRIPITON_ROBOT_DESCRIPTION_H_

#include <string>
#include <unordered_map>

#include "robotics-common/robot/description/joint_description.h"
#include "robotics-common/robot/description/link_description.h"

namespace robotics_common {
namespace robot_description {

/// @class RobotDescription
class RobotDescription {
 public:
  RobotDescription(const std::string& name);

  LinkDescription* AddLink(const std::string& link_name);
  JointDescription* AddJoint(const std::string& joing_name);

 private:
  std::string name_;

  std::unordered_map<std::string, LinkDescription> link_descriptions_;
  std::unordered_map<std::string, JointDescription> joint_descriptions_;
};

}  // namespace robot_description
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_ROBOT_DESCRIPITON_ROBOT_DESCRIPTION_H_
