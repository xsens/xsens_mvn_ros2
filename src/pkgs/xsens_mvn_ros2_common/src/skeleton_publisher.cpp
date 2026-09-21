// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#include "xsens_mvn_ros2_common/skeleton_publisher.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace xsens_mvn_ros2
{

SkeletonPublisher::SkeletonPublisher(
  rclcpp_lifecycle::LifecycleNode & node, tf2_ros::TransformBroadcaster & tf_broadcaster,
  const std::string & model_name, const std::string & topic_prefix,
  bool publish_joint_states)
: node_(node), tf_broadcaster_(tf_broadcaster), model_name_override_(model_name)
{
  const std::string prefix = topic_prefix.empty() ? "" : topic_prefix + "/";
  if (publish_joint_states) {
    joint_state_pub_ = node_.create_publisher<sensor_msgs::msg::JointState>(
      prefix + "joint_states", rclcpp::SystemDefaultsQoS());
  }
  link_state_pub_ = node_.create_publisher<xsens_mvn_msgs::msg::LinkStateArray>(
    prefix + "link_states", rclcpp::SystemDefaultsQoS());
  cacheParameters();
}

void SkeletonPublisher::cacheParameters()
{
  cached_model_name_ = model_name_override_.empty() ?
    node_.get_parameter("model_name").as_string() :
    model_name_override_;
  cached_reference_frame_ = node_.get_parameter("reference_frame").as_string();
}

void SkeletonPublisher::publishLinkStates(
  const rclcpp::Time & stamp,
  const std::unordered_map<std::string, SegmentKinematics> & segments)
{
  const auto & model_name = cached_model_name_;
  const auto & reference_frame = cached_reference_frame_;
  const bool want_link_states = link_state_pub_->get_subscription_count() > 0;

  xsens_mvn_msgs::msg::LinkStateArray link_state_array;
  link_state_array.header.stamp = stamp;

  for (const auto & [segment, kin] : segments) {
    const std::string & parent_name = kineticParent(segment);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.child_frame_id = frameName(model_name, segment);

    if (parent_name.empty()) {
      // Root segment (pelvis): absolute pose in world frame.
      if (kin.orientation.isApprox(Eigen::Quaterniond::Identity())) {
        RCLCPP_DEBUG(
          node_.get_logger(),
          "Skipping root segment '%s': orientation is identity (no data yet).", segment.c_str());
        continue;
      }
      tf.header.frame_id = reference_frame;
      tf.transform.translation.x = kin.position.x();
      tf.transform.translation.y = kin.position.y();
      tf.transform.translation.z = kin.position.z();
      tf.transform.rotation.x = kin.orientation.x();
      tf.transform.rotation.y = kin.orientation.y();
      tf.transform.rotation.z = kin.orientation.z();
      tf.transform.rotation.w = kin.orientation.w();
    } else {
      // Non-root: pose relative to kinematic parent.
      const auto parent_it = segments.find(parent_name);
      if (parent_it == segments.end()) {
        RCLCPP_DEBUG(
          node_.get_logger(),
          "Skipping segment '%s': parent '%s' not found in segments map.",
          segment.c_str(), parent_name.c_str());
        continue;
      }
      const auto & parent_kin = parent_it->second;
      if (parent_kin.orientation.isApprox(Eigen::Quaterniond::Identity())) {
        RCLCPP_DEBUG(
          node_.get_logger(),
          "Skipping segment '%s': parent '%s' has identity orientation (no data yet).",
          segment.c_str(), parent_name.c_str());
        continue;
      }
      if (kin.orientation.isApprox(Eigen::Quaterniond::Identity())) {
        RCLCPP_DEBUG(
          node_.get_logger(),
          "Skipping segment '%s': orientation is identity (no data yet).", segment.c_str());
        continue;
      }

      const Eigen::Quaterniond q_rel = parent_kin.orientation.inverse() * kin.orientation;
      const Eigen::Vector3d p_rel =
        parent_kin.orientation.inverse().toRotationMatrix() * (kin.position - parent_kin.position);

      tf.header.frame_id = frameName(model_name, parent_name);
      tf.transform.translation.x = p_rel.x();
      tf.transform.translation.y = p_rel.y();
      tf.transform.translation.z = p_rel.z();
      tf.transform.rotation.x = q_rel.x();
      tf.transform.rotation.y = q_rel.y();
      tf.transform.rotation.z = q_rel.z();
      tf.transform.rotation.w = q_rel.w();
    }
    tf_broadcaster_.sendTransform(tf);

    if (!want_link_states) {
      continue;
    }
    xsens_mvn_msgs::msg::LinkState ls;
    ls.header.stamp = stamp;
    ls.header.frame_id = tf.child_frame_id;
    ls.pose.position = tf2::toMsg(kin.position);
    ls.pose.orientation = tf2::toMsg(kin.orientation);
    ls.twist.linear.x = kin.velocity_linear.x();
    ls.twist.linear.y = kin.velocity_linear.y();
    ls.twist.linear.z = kin.velocity_linear.z();
    ls.twist.angular.x = kin.velocity_angular.x();
    ls.twist.angular.y = kin.velocity_angular.y();
    ls.twist.angular.z = kin.velocity_angular.z();
    ls.accel.linear.x = kin.accel_linear.x();
    ls.accel.linear.y = kin.accel_linear.y();
    ls.accel.linear.z = kin.accel_linear.z();
    ls.accel.angular.x = kin.accel_angular.x();
    ls.accel.angular.y = kin.accel_angular.y();
    ls.accel.angular.z = kin.accel_angular.z();
    link_state_array.states.push_back(ls);
  }

  if (want_link_states) {
    link_state_pub_->publish(link_state_array);
  }
}

void SkeletonPublisher::publishJointStates(
  const rclcpp::Time & stamp, const std::vector<JointAngles> & joints)
{
  if (!joint_state_pub_ || joint_state_pub_->get_subscription_count() == 0) {
    return;
  }
  const auto & model_name = cached_model_name_;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  for (const auto & j : joints) {
    msg.name.push_back(frameName(model_name, j.name) + "_x");
    msg.name.push_back(frameName(model_name, j.name) + "_y");
    msg.name.push_back(frameName(model_name, j.name) + "_z");
    msg.position.push_back(j.x);
    msg.position.push_back(j.y);
    msg.position.push_back(j.z);
  }
  joint_state_pub_->publish(msg);
}

void SkeletonPublisher::activate()
{
  link_state_pub_->on_activate();
  if (joint_state_pub_) {
    joint_state_pub_->on_activate();
  }
}

void SkeletonPublisher::deactivate()
{
  link_state_pub_->on_deactivate();
  if (joint_state_pub_) {
    joint_state_pub_->on_deactivate();
  }
}

}  // namespace xsens_mvn_ros2
