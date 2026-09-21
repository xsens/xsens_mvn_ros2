// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <xsens_mvn_ros2_common/i_motion_capture_source.hpp>
#include <xsens_mvn_ros2_common/skeleton_publisher.hpp>
#include <xsens_mvn_ros2_stream/xsens_stream_node_parameters.hpp>

namespace xsens_mvn_ros2
{

class XsensStreamNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  XsensStreamNode();
  ~XsensStreamNode() final;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override;

private:
  bool initialize_xsens_stream_client();
  void publish();
  void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::shared_ptr<xsens_stream_node::ParamListener> m_paramListener;
  std::unique_ptr<IMotionCaptureSource> m_xsensClient;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
  /// One publisher set per avatar, created as avatars are discovered.
  std::map<uint8_t, std::unique_ptr<SkeletonPublisher>> m_skeletonPubs;
  std::map<uint8_t,
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Point>::SharedPtr> m_comPubs;
  /// TF prefix / topic namespace chosen for each avatar.
  std::map<uint8_t, std::string> m_avatarNames;
  bool m_publishersActive{false};

  /// Name for @p avatar_id: an explicit avatar_names entry if given, else
  /// model_name for the primary avatar and "<model_name>_<id>" for the rest.
  std::string avatarName(uint8_t avatar_id);
  /// Creates (and activates, if the node is active) this avatar's publishers.
  void ensureAvatarPublishers(uint8_t avatar_id);
  std::unique_ptr<diagnostic_updater::Updater> m_diagnosticUpdater;
  // timers
  rclcpp::TimerBase::SharedPtr m_updateTimer;
  rclcpp::TimerBase::SharedPtr m_diagnosticTimer;
  // publishers
  // staleness tracking
  std::atomic<int64_t> m_lastDataTimeNs{0};
};

}  // namespace xsens_mvn_ros2
