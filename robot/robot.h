#ifndef ROBOTICS_COMMON_ROBOT_ROBOT_H_
#define ROBOTICS_COMMON_ROBOT_ROBOT_H_

#include <memory>
#include <unordered_map>

#include "robotics-common/math/frame_manager.h"
#include "robotics-common/math/transform.h"
#include "robotics-common/robot/joint.h"
#include "robotics-common/robot/rigid_body.h"

namespace robotics_common {
namespace robot {

class Robot {
 public:
  struct LinkData {
    RigidBody link_properties;
    math::FrameNode* body_frame_node;
    std::vector<FixedJoint> link_frame_transforms;
  };

  using LinkMap = std::unordered_map<std::string, LinkData>;
  using JointMap = std::unordered_map<std::string, Joint*>;

  class Builder {
   public:
    Builder(const std::string& robot_name);
    Builder(const std::string& robot_name, const std::string& root_link_name);

    Builder&& AddLink(std::unique_ptr<RigidBody> rigid_body,
                      const std::string& parent_joint_name,
                      const math::Transform& transform_in_parent);

    Builder&& AddJoint(const std::string& parent_link_name,
                       std::unique_ptr<Joint> joint,
                       const math::Transform& transform_in_parent);

    Builder&& AddConstraint(
        const std::pair<std::string, math::Transform>& first_link_info,
        const std::pair<std::string, math::Transform>& second_link_info,
        std::unique_ptr<Constraint> constraint);

    common::ErrorOr<Robot> Construct();

   private:
    friend class Robot;

    Builder&& This() { return std::move(*this); }

    std::string name_;
    Joint* root_joint_;
    RigidBody root_link_;

    std::vector<std::unique_ptr<Joint>> joints_;
    JointMap joint_map_;

    LinkMap link_map_;

    math::FrameManager frame_manager_;

    bool has_error_;
  };

 private:
  Robot(Builder&& builder);

  // Joint connecting the robot to the world
  Joint* root_joint_;

  LinkMap link_map_;
  std::vector<std::unique_ptr<Joint>> joints_;
  JointMap joint_map_;

  math::FrameManager frame_manager_;
};

}  // namespace robot
}  // namespace robotics_common

#endif  // ROBOTICS_COMMON_ROBOT_ROBOT_H_
