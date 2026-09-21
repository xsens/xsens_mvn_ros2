// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <xsens_mvn_sdk/parsermanager.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/thread.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <xsens_mvn_ros2_common/i_motion_capture_source.hpp>
#include <xsens_mvn_ros2_stream/human_data_handler.hpp>
#include <xsens_mvn_ros2_stream/socket.hpp>

constexpr int MAX_MVN_DATAGRAM_SIZE = 5000;

/// Byte offset of the avatar id within an MVN datagram header, which is laid
/// out as: "MXTPxx" (6) + sample counter (4) + datagram counter (1) +
/// item count (1) + frame time (4), then the avatar id.
constexpr int MVN_AVATAR_ID_OFFSET = 16;

/// Per-avatar parsing and model state.
///
/// MVN streams every avatar on one port - the captured subject(s) plus any
/// tracked objects - tagged by an avatar id in each datagram header.  Each one
/// needs its own parser and model, because a single shared parser holds only
/// the most recent datagram and one avatar's pose would overwrite another's.
struct AvatarStream
{
  ParserManager parser;
  xsens_mvn_ros2::HumanDataHandler::Ptr data;
  std::vector<std::string> linkNames;
  std::vector<std::string> jointNames;
  bool linksBuilt{false};
  bool jointsBuilt{false};
  /// True for a rigid body (an object/prop): fewer segments than a body model
  /// and no joint-angle datagram of its own.
  bool isObject{false};
  std::atomic<int64_t> lastDataNs{0};
};

class XsensStreamClient : public xsens_mvn_ros2::IMotionCaptureSource
{
public:
  XsensStreamClient(
    rclcpp::Logger logger, const int & udp_port, const int & avatar_id = 0,
    bool track_all_avatars = false);
  ~XsensStreamClient() override;
  bool init();

  // IMotionCaptureSource interface — reports the primary avatar.
  std::unordered_map<std::string, xsens_mvn_ros2::SegmentKinematics> getSegments() const override;
  std::vector<xsens_mvn_ros2::JointAngles> getJoints() const override;
  std::optional<Eigen::Vector3d> getCOM() const override;

  // ---- Multi-avatar access ----

  /// Ids of every avatar discovered so far whose model is ready to publish.
  std::vector<uint8_t> avatarIds() const override;
  std::unordered_map<std::string, xsens_mvn_ros2::SegmentKinematics> getAvatarSegments(
    uint8_t avatar_id) const override;
  std::vector<xsens_mvn_ros2::JointAngles> getAvatarJoints(uint8_t avatar_id) const override;
  std::optional<Eigen::Vector3d> getAvatarCOM(uint8_t avatar_id) const override;
  /// Nanoseconds (steady clock) of this avatar's last datagram, or 0 if unknown.
  int64_t avatarLastDataTimeNs(uint8_t avatar_id) const override;
  /// True if this avatar is a rigid object rather than a body model.
  bool isObject(uint8_t avatar_id) const override;
  /// The avatar reported through the IMotionCaptureSource interface.
  uint8_t primaryAvatarId() const {return m_avatarId;}
  int64_t lastDataTimeNs() const override
  {
    return m_lastDataTimeNs;
  }
  bool isActive() const override
  {
    return m_clientActive;
  }

private:
  rclcpp::Logger m_logger;
  int m_udpPort;
  uint8_t m_avatarId;
  bool m_trackAllAvatars;
  std::shared_ptr<Socket> m_udpSocket;
  /// Avatar id -> its parser and model.  Held by pointer because ParserManager
  /// owns a raw datagram pointer and must never be copied.
  std::map<uint8_t, std::unique_ptr<AvatarStream>> m_avatars;
  /// Avatar whose datagram was parsed by the most recent readData().
  uint8_t m_lastAvatarId{0};

  boost::thread m_dataAcquisitionThread;
  void dataAcquisitionCallback();
  char m_dataBuffer[MAX_MVN_DATAGRAM_SIZE];
  std::atomic<bool> m_clientActive{false};
  mutable std::mutex m_dataMutex;
  std::atomic<int64_t> m_lastDataTimeNs{0};

  /// Returns the stream for @p avatar_id, creating it on first sight.
  AvatarStream & avatarStream(uint8_t avatar_id);
  /// Returns the stream for @p avatar_id, or nullptr if never seen.
  const AvatarStream * findAvatar(uint8_t avatar_id) const;

  bool buildXsensModel();
  /// Builds whatever part of @p av's model the just-parsed datagram allows:
  /// links from a quaternion datagram, joints from a joint-angles datagram.
  /// Safe to call on every datagram; does nothing once the model is complete.
  void buildAvatarModelIncremental(AvatarStream & av, uint8_t avatar_id);
  bool buildLinksFromQuaternions(AvatarStream & av, QuaternionDatagram & quaternions);
  bool buildJointsFromAngles(AvatarStream & av, JointAnglesDatagram & joint_angles);
  QuaternionDatagram waitForQuaternionDatagram();
  JointAnglesDatagram waitForJointAnglesDatagram();

  bool readData();
  void updateJointAngles(AvatarStream & av);
  void updateLinkPoses(AvatarStream & av);
  void updateLinkLinearTwists(AvatarStream & av);
  void updateLinkAngularTwists(AvatarStream & av);
  void updateCOM(AvatarStream & av);

  Eigen::Vector3d jointAngleToEigenVector3d(
    const JointAngle & joint_angle, const double & x_axis, const double & y_axis,
    const double & z_axis) const;
  void rotateLink(
    AvatarStream & av, const std::string & link_name, const Eigen::Quaterniond & quat) const;
};
