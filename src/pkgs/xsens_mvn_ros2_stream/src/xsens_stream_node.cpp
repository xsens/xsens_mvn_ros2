// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <xsens_mvn_ros2_common/skeleton_publisher.hpp>
#include <xsens_mvn_ros2_common/xsens_model.hpp>
#include <xsens_mvn_ros2_stream/xsens_stream_client.hpp>
#include <xsens_mvn_ros2_stream/xsens_stream_node.hpp>

namespace xsens_mvn_ros2
{

XsensStreamNode::XsensStreamNode()
: rclcpp_lifecycle::LifecycleNode("xsens_mvn_ros2_stream_node")
{
}

XsensStreamNode::~XsensStreamNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XsensStreamNode::on_configure(const rclcpp_lifecycle::State &)
{
  m_paramListener = std::make_shared<xsens_stream_node::ParamListener>(
    get_node_parameters_interface());

  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // Publishers are created per avatar as they are discovered; the primary one
  // is made up front so its topics exist from configure, as before.
  ensureAvatarPublishers(
    static_cast<uint8_t>(m_paramListener->get_params().avatar_id));

  m_diagnosticUpdater = std::make_unique<diagnostic_updater::Updater>(this);
  m_diagnosticUpdater->setHardwareID("Xsens MVN Stream");
  m_diagnosticUpdater->add("Stream status", this, &XsensStreamNode::diagnosticsCallback);
  m_diagnosticTimer = create_wall_timer(
    std::chrono::seconds(1), [this]() {m_diagnosticUpdater->force_update();});

  initialize_xsens_stream_client();

  RCLCPP_INFO(get_logger(), "Configured. Waiting for activation...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XsensStreamNode::on_activate(const rclcpp_lifecycle::State &)
{
  m_publishersActive = true;
  for (auto & [id, pub] : m_skeletonPubs) {
    pub->activate();
    m_comPubs[id]->on_activate();
  }

  const auto params = m_paramListener->get_params();
  m_updateTimer = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / params.update_frequency)),
    [this]() {
      if ((!m_xsensClient || !m_xsensClient->isActive()) &&
      !initialize_xsens_stream_client())
      {
        return;
      }
      publish();
    });

  RCLCPP_INFO(get_logger(), "Activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XsensStreamNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (m_updateTimer) {
    m_updateTimer->cancel();
    m_updateTimer.reset();
  }
  m_publishersActive = false;
  for (auto & [id, pub] : m_skeletonPubs) {
    pub->deactivate();
    m_comPubs[id]->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XsensStreamNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  m_updateTimer.reset();
  m_diagnosticTimer.reset();
  m_diagnosticUpdater.reset();
  m_xsensClient.reset();
  m_skeletonPubs.clear();
  m_comPubs.clear();
  m_avatarNames.clear();
  m_publishersActive = false;
  m_tfBroadcaster.reset();
  m_paramListener.reset();
  m_lastDataTimeNs = 0;

  RCLCPP_INFO(get_logger(), "Cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XsensStreamNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  m_updateTimer.reset();
  m_diagnosticTimer.reset();
  m_diagnosticUpdater.reset();
  m_xsensClient.reset();
  m_skeletonPubs.clear();
  m_comPubs.clear();
  m_avatarNames.clear();
  m_publishersActive = false;
  m_tfBroadcaster.reset();
  m_paramListener.reset();

  RCLCPP_INFO(get_logger(), "Shut down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
XsensStreamNode::on_error(const rclcpp_lifecycle::State &)
{
  m_updateTimer.reset();
  m_diagnosticTimer.reset();
  m_diagnosticUpdater.reset();
  m_xsensClient.reset();
  m_skeletonPubs.clear();
  m_comPubs.clear();
  m_avatarNames.clear();
  m_publishersActive = false;
  m_tfBroadcaster.reset();
  m_paramListener.reset();
  m_lastDataTimeNs = 0;

  RCLCPP_ERROR(get_logger(), "Error encountered, returning to unconfigured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool XsensStreamNode::initialize_xsens_stream_client()
{
  // Reset a dead client so a fresh one is created below.
  if (m_xsensClient && !m_xsensClient->isActive()) {
    RCLCPP_WARN(get_logger(), "Stream client lost connection. Reconnecting...");
    m_xsensClient.reset();
  }
  if (!m_xsensClient) {
    try {
      auto client =
        std::make_unique<XsensStreamClient>(
        get_logger(), m_paramListener->get_params().udp_port,
        m_paramListener->get_params().avatar_id,
        m_paramListener->get_params().track_all_avatars);
      if (!client->init()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *get_clock(), 5000, "Xsens client initialization failed; retrying.");
        return false;
      }
      m_xsensClient = std::move(client);
    } catch (const std::exception & err) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 5000, "Could not set up Xsens client: %s", err.what());
      return false;
    }
  }
  return true;
}

std::string XsensStreamNode::avatarName(uint8_t avatar_id)
{
  auto it = m_avatarNames.find(avatar_id);
  if (it != m_avatarNames.end()) {
    return it->second;
  }
  const auto params = m_paramListener->get_params();
  std::string name;
  if (avatar_id < params.avatar_names.size() && !params.avatar_names[avatar_id].empty()) {
    name = params.avatar_names[avatar_id];
  } else if (avatar_id == static_cast<uint8_t>(params.avatar_id)) {
    name = params.model_name;
  } else {
    name = params.model_name + "_" + std::to_string(static_cast<unsigned>(avatar_id));
  }
  m_avatarNames[avatar_id] = name;
  return name;
}

void XsensStreamNode::ensureAvatarPublishers(uint8_t avatar_id)
{
  if (m_skeletonPubs.count(avatar_id) > 0) {
    return;
  }
  const std::string name = avatarName(avatar_id);
  // The primary avatar keeps the bare topic names so a single-suit setup sees
  // exactly the topics it always had; additional avatars are namespaced.
  const bool is_primary =
    avatar_id == static_cast<uint8_t>(m_paramListener->get_params().avatar_id);
  const std::string topic_prefix = is_primary ? "" : name;

  // A tracked object has no joints, so it should not advertise a joint_states
  // topic that could only ever carry an empty message.  The primary avatar is
  // created before the client exists, so it keeps the topic: its kind is not
  // known yet, and a suit is the overwhelmingly common case.
  const bool has_joints = !(m_xsensClient && m_xsensClient->isObject(avatar_id));
  auto skeleton = std::make_unique<SkeletonPublisher>(
    *this, *m_tfBroadcaster, name, topic_prefix, has_joints);
  auto com = create_publisher<geometry_msgs::msg::Point>(
    (topic_prefix.empty() ? "com" : topic_prefix + "/com"), rclcpp::SystemDefaultsQoS());

  if (m_publishersActive) {
    skeleton->activate();
    com->on_activate();
  }
  m_skeletonPubs[avatar_id] = std::move(skeleton);
  m_comPubs[avatar_id] = com;

  if (!is_primary) {
    RCLCPP_INFO(
      get_logger(), "Publishing avatar %u as '%s' (topics under %s/).",
      static_cast<unsigned>(avatar_id), name.c_str(), topic_prefix.c_str());
  }
}

void XsensStreamNode::publish()
{
  const auto stamp = now();
  const auto params = m_paramListener->get_params();
  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto stale_ns = static_cast<int64_t>(params.avatar_stale_timeout * 1e9);

  m_lastDataTimeNs = m_xsensClient->lastDataTimeNs();

  for (const uint8_t avatar_id : m_xsensClient->avatarIds()) {
    // An avatar that stopped sending would otherwise keep its last pose alive
    // in TF forever; drop it until data resumes.
    const int64_t last = m_xsensClient->avatarLastDataTimeNs(avatar_id);
    if (last > 0 && (now_ns - last) > stale_ns) {
      continue;
    }
    ensureAvatarPublishers(avatar_id);

    auto & skeleton_pub = m_skeletonPubs[avatar_id];
    skeleton_pub->publishLinkStates(stamp, m_xsensClient->getAvatarSegments(avatar_id));
    skeleton_pub->publishJointStates(stamp, m_xsensClient->getAvatarJoints(avatar_id));

    auto & com_pub = m_comPubs[avatar_id];
    if (com_pub->get_subscription_count() > 0) {
      if (auto com = m_xsensClient->getAvatarCOM(avatar_id)) {
        com_pub->publish(toMsg(*com));
      }
    }
  }
}

void XsensStreamNode::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!m_xsensClient) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Stream client not initialized.");
    return;
  }

  if (!m_xsensClient->isActive()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Stream client disconnected.");
    return;
  }

  // Base status: connected and running.
  stat.summary(
    diagnostic_msgs::msg::DiagnosticStatus::OK, "Stream client connected and receiving data.");

  stat.add("UDP Port", m_paramListener->get_params().udp_port);
  stat.add("Avatar ID", m_paramListener->get_params().avatar_id);

  stat.add("Links", static_cast<int>(m_xsensClient->getSegments().size()));
  stat.add("Joints", static_cast<int>(m_xsensClient->getJoints().size()));

  // Per-avatar staleness.  publish() already drops an avatar that goes quiet,
  // so without this an avatar could silently vanish from TF while the overall
  // status stayed green on the strength of the other avatars' data.
  const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto stale_ns =
    static_cast<int64_t>(m_paramListener->get_params().avatar_stale_timeout * 1e9);

  const auto avatar_ids = m_xsensClient->avatarIds();
  stat.add("Avatars tracked", static_cast<int>(avatar_ids.size()));

  std::vector<std::string> stale_avatars;
  for (const uint8_t id : avatar_ids) {
    const std::string name = avatarName(id);
    std::string detail = name + (m_xsensClient->isObject(id) ? " (object)" : " (body)");

    const int64_t last = m_xsensClient->avatarLastDataTimeNs(id);
    if (last > 0) {
      const int64_t age_ns = now_ns - last;
      detail += ", last data " + std::to_string(age_ns / 1'000'000) + " ms ago";
      if (age_ns > stale_ns) {
        detail += " [STALE, not being published]";
        stale_avatars.push_back(name);
      }
    } else {
      detail += ", no data yet";
    }
    stat.add("Avatar " + std::to_string(static_cast<unsigned>(id)), detail);
  }

  if (!stale_avatars.empty()) {
    std::string names;
    for (size_t i = 0; i < stale_avatars.size(); ++i) {
      names += (i > 0 ? ", " : "") + stale_avatars[i];
    }
    stat.mergeSummary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Stale avatar(s) dropped from TF: " + names + ".");
  }

  // Staleness check: warn if no data arrived within 2× the expected period.
  if (m_lastDataTimeNs > 0) {
    auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    auto elapsed_ms = (now_ns - m_lastDataTimeNs) / 1'000'000;
    stat.add("Last data received (ms ago)", static_cast<int>(elapsed_ms));
    int expected_period_ms = 1000 / m_paramListener->get_params().update_frequency;
    if (elapsed_ms > 2 * expected_period_ms) {
      stat.mergeSummary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "No data received in the last " + std::to_string(elapsed_ms) + " ms.");
    }
  }
}

}  // namespace xsens_mvn_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xsens_mvn_ros2::XsensStreamNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
