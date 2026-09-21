// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <Eigen/Geometry>

#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <xsens_mvn_msgs/msg/link_state_array.hpp>
#include <xsens_mvn_ros2_common/xsens_model.hpp>

namespace xsens_mvn_ros2
{

/// Kinematic state for a single body segment, in absolute (world) coordinates.
/// Arm corrections should be applied before passing data to SkeletonPublisher.
struct SegmentKinematics
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d velocity_linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_angular = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_angular = Eigen::Vector3d::Zero();
};

/// Joint angles for a single joint (3 DOF, in radians).
struct JointAngles
{
  std::string name;
  double x = 0.0, y = 0.0, z = 0.0;
};

// ---- Arm correction constants (single source of truth) ----

/// ±90 deg post-rotation around X to align arm local frames between
/// the MVN network protocol / XME SDK and the URDF/TF convention.
inline const Eigen::Quaterniond & rightArmCorrection()
{
  static const Eigen::Quaterniond q(0.7071068, -0.7071068, 0.0, 0.0);
  return q;
}
inline const Eigen::Quaterniond & leftArmCorrection()
{
  static const Eigen::Quaterniond q(0.7071068, 0.7071068, 0.0, 0.0);
  return q;
}

/// Apply arm orientation correction if the segment is an arm segment.
inline Eigen::Quaterniond applyArmCorrection(
  const std::string & segment, const Eigen::Quaterniond & q)
{
  if (segment == "right_upper_arm" || segment == "right_forearm" || segment == "right_hand") {
    return q * rightArmCorrection();
  }
  if (segment == "left_upper_arm" || segment == "left_forearm" || segment == "left_hand") {
    return q * leftArmCorrection();
  }
  return q;
}

/// Build a fully-qualified TF frame name: "{model_name}_{segment}".
inline std::string frameName(const std::string & model, const std::string & segment)
{
  return model + "_" + segment;
}

/// Shared publisher for skeleton TF transforms, LinkStateArray and JointState messages.
/// Both the stream node and XME node delegate their ROS publishing to this class.
class SkeletonPublisher
{
public:
  /// @param node            Owning lifecycle node.
  /// @param tf_broadcaster   Shared TF broadcaster.
  /// @param model_name       TF frame prefix for this model.  Empty means "use
  ///                         the node's model_name parameter", which is what a
  ///                         single-model node wants.
  /// @param topic_prefix     Sub-namespace for link_states/joint_states.  Empty
  ///                         publishes on the bare topic names.  Multi-avatar
  ///                         nodes give each non-primary avatar its own prefix
  ///                         so the topics do not collide.
  /// @param publish_joint_states  False for a model that has no joints (a
  ///                         tracked object), so it does not advertise a
  ///                         joint_states topic that could only ever be empty.
  SkeletonPublisher(
    rclcpp_lifecycle::LifecycleNode & node, tf2_ros::TransformBroadcaster & tf_broadcaster,
    const std::string & model_name = "", const std::string & topic_prefix = "",
    bool publish_joint_states = true);

  /// Publish TF transforms (relative parent-child) and LinkStateArray for all segments.
  /// @param stamp       Message timestamp.
  /// @param segments    Map of canonical segment name -> kinematics (arm corrections pre-applied).
  void publishLinkStates(
    const rclcpp::Time & stamp,
    const std::unordered_map<std::string, SegmentKinematics> & segments);

  /// Publish a JointState message.
  /// @param stamp   Message timestamp.
  /// @param joints  Vector of joint angles (already in radians).
  void publishJointStates(
    const rclcpp::Time & stamp, const std::vector<JointAngles> & joints);

  /// Activate lifecycle publishers. Call from the owning node's on_activate.
  void activate();

  /// Deactivate lifecycle publishers. Call from the owning node's on_deactivate.
  void deactivate();

  /// Cache parameter values. Call after parameters are declared and before publishing.
  void cacheParameters();

private:
  rclcpp_lifecycle::LifecycleNode & node_;
  tf2_ros::TransformBroadcaster & tf_broadcaster_;
  rclcpp_lifecycle::LifecyclePublisher<xsens_mvn_msgs::msg::LinkStateArray>::SharedPtr
    link_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  // Cached parameters to avoid per-publish string allocation
  std::string cached_model_name_;
  std::string cached_reference_frame_;
  // Non-empty when the model name was supplied explicitly rather than read
  // from the node parameter.
  std::string model_name_override_;
};

}  // namespace xsens_mvn_ros2
