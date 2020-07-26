#include "robotics-common/robot/robot.h"

#include <utility>

namespace robotics_common {
namespace robot {

namespace {

std::string LinkFrameNameFromLinkName(const std::string& link_name) {
  return link_name + "Frame";
}

std::string BeforeJointFrameNameFromJointName(const std::string& joint_name) {
  return joint_name + "BeforeFrame";
}

std::string AfterJointFrameNameFromJointName(const std::string& joint_name) {
  return joint_name + "AfterFrame";
}

std::string RootJointNameFromRobotName(const std::string& robot_name) {
  return robot_name + "RootJoint";
}

}  // namespace

using Builder = Robot::Builder;

Builder::Builder(const std::string& robot_name)
    : Builder(robot_name, robot_name + "Root") {}

Builder::Builder(const std::string& robot_name,
                 const std::string& root_link_name)
    : name_(robot_name),
      root_joint_(new SixDofJoint(RootJointNameFromRobotName(robot_name))),
      root_link_(root_link_name),
      has_error_(false) {
  // Create a root joint that connects this robot to the world frame
  joints_.emplace_back(std::unique_ptr<Joint>(root_joint_));
  joint_map_.insert({root_joint_->name(), root_joint_});

  // Create a frame for the root link of this robot
  common::ErrorOr<math::FrameNode*> root_link_frame =
      frame_manager_.CreateFrameRelativeToWorld(
          LinkFrameNameFromLinkName(root_link_.name()), root_joint_);

  if (root_link_frame.HasValue()) {
    link_map_.insert(
        {root_link_.name(),
         LinkData{.link_properties = Link(root_link_.name()),
                  .body_frame_node = root_link_frame.ValueOrDie()}});
  } else {
    has_error_ = true;
  }
}

Builder&& Builder::AddLinkWithRevoluteJoint(
    const std::string& link_name, const std::string& joint_name,
    const std::pair<std::string, math::Transform>& parent_link_info,
    math::Axis3d axis) {
  // Check to make sure that the joint has not already been created
  JointMap::iterator joint_it = joint_map_.find(joint_name);
  if (joint_it != joint_map_.end()) {
    has_error_ = true;
    return This();
  }

  // Check to make sure that link with the same name does not exist
  LinkMap::iterator link_it = link_map_.find(link_name);
  if (link_it != link_map_.end()) {
    has_error_ = true;
    return This();
  }

  std::string link_frame_name =
      LinkFrameNameFromLinkName(parent_link_info.first);

  // Find the body frame for the parent link
  LinkMap::iterator parent_frame_it = link_map_.find(link_frame_name);
  if (parent_frame_it == link_map_.end()) {
    has_error_ = true;
    return This();
  }

  std::unique_ptr<RevoluteJoint> joint =
      std::make_unique<RevoluteJoint>(joint_name, axis);
  // Insert the joint into the map and move ownership of joint to the vector
  joint_map_.insert({joint_name, joint.get()});
  joints_.emplace_back(std::move(joint));

  // Create the frame before the joint
  common::ErrorOr<math::FrameNode*> before_joint_frame_result =
      frame_manager_.CreateFrame(BeforeJointFrameNameFromJointName(joint_name),
                                 parent_frame_it->first, nullptr);
  if (!before_joint_frame_result.HasValue()) {
    has_error_ = true;
    return This();
  }
  math::FrameNode* before_joint_frame = before_joint_frame_result.ValueOrDie();

  // Create the frame after the joint
  common::ErrorOr<math::FrameNode*> after_joint_frame_result =
      frame_manager_.CreateFrame(AfterJointFrameNameFromJointName(joint_name),
                                 before_joint_frame->name(), joint.get());
  if (!after_joint_frame_result.HasValue()) {
    has_error_ = true;
    return This();
  }
  math::FrameNode* after_joint_frame = after_joint_frame_result.ValueOrDie();

  // Create the body frame for the new link
  common::ErrorOr<math::FrameNode*> body_frame_result =
      frame_manager_.CreateFrame(link_frame_name, after_joint_frame->name(),
                                 nullptr);
  if (!body_frame_result.HasValue()) {
    has_error_ = true;
    return This();
  }

  // Create the link
  link_map_.insert(
      {link_name, LinkData{.link_properties = Link(link_name),
                           .body_frame_node = body_frame_result.ValueOrDie()}});

  return This();
}

Builder&& Builder::AddJoint(const std::string& parent_link_name,
                            std::unique_ptr<Joint> joint,
                            const math::Transform& transform_in_parent) {
  has_error_ = true;
  return This();
}

common::ErrorOr<Robot> Builder::Construct() {
  if (has_error_) {
    return common::Error::kInvalidArgument;
  }
  return Robot(This());
}

Robot::Robot(Robot::Builder&& builder)
    : root_joint_(builder.root_joint_),
      link_map_(std::move(builder.link_map_)),
      joints_(std::move(builder.joints_)),
      joint_map_(builder.joint_map_),
      frame_manager_(std::move(builder.frame_manager_)) {}

}  // namespace robot
}  // namespace robotics_common
