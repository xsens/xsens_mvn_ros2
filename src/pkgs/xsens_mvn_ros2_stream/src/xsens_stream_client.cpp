// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#include "xsens_mvn_ros2_stream/xsens_stream_client.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <system_error>

#include <xsens_mvn_ros2_common/xsens_model.hpp>

XsensStreamClient::XsensStreamClient(
  rclcpp::Logger logger, const int & udp_port, const int & avatar_id, bool track_all_avatars)
: m_logger(logger), m_udpPort(udp_port), m_avatarId(static_cast<uint8_t>(avatar_id)),
  m_trackAllAvatars(track_all_avatars), m_lastAvatarId(static_cast<uint8_t>(avatar_id))
{}

AvatarStream & XsensStreamClient::avatarStream(uint8_t avatar_id)
{
  auto it = m_avatars.find(avatar_id);
  if (it == m_avatars.end()) {
    it = m_avatars.emplace(avatar_id, std::make_unique<AvatarStream>()).first;
    it->second->data = std::make_shared<xsens_mvn_ros2::HumanDataHandler>(m_logger);
  }
  return *it->second;
}

const AvatarStream * XsensStreamClient::findAvatar(uint8_t avatar_id) const
{
  auto it = m_avatars.find(avatar_id);
  return (it == m_avatars.end()) ? nullptr : it->second.get();
}

bool XsensStreamClient::init()
{
  // Initialize UDP communication
  m_udpSocket = std::make_shared<Socket>(IP_UDP);
  if (!m_udpSocket->bind(static_cast<uint16_t>(m_udpPort))) {
    RCLCPP_ERROR(
      m_logger,
      "Error binding Xsens UDP port %d. "
      "Check: (1) Is another process already using this port? "
      "(2) Is MVN Studio configured to stream on this port?",
      m_udpPort);
    return false;
  }

  m_dataAcquisitionThread = boost::thread(&XsensStreamClient::dataAcquisitionCallback, this);

  // Wait for Xsens data acquisition activation (with timeout)
  auto init_start = std::chrono::steady_clock::now();
  while (!m_clientActive) {
    if (m_acquisitionEnded) {
      // The model build failed outright (socket error, malformed datagram);
      // the thread has already logged why.
      return false;
    }
    if (std::chrono::steady_clock::now() - init_start > std::chrono::seconds(30)) {
      RCLCPP_ERROR(
        m_logger,
        "Timeout waiting for data acquisition after 30 seconds. "
        "Check: (1) Is MVN Studio streaming? "
        "(2) Is the correct UDP port (%d) configured in MVN Studio? "
        "(3) Is a firewall blocking UDP traffic?",
        m_udpPort);
      return false;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
  RCLCPP_INFO(m_logger, "Xsens stream client initialized.");

  return true;
}

void XsensStreamClient::dataAcquisitionCallback()
{
  RCLCPP_INFO(m_logger, "Xsens stream client starting data acquisition.");

  if (!buildXsensModel()) {
    RCLCPP_ERROR(
      m_logger,
      "Failure building human model. Data acquisition stopped. "
      "Check: (1) Is MVN Studio streaming quaternion and joint angle datagrams? "
      "(2) Is the MVN datagram format compatible?");
    m_clientActive = false;
  } else {
    RCLCPP_INFO(m_logger, "Human model built successfully.");
    m_clientActive = true;
  }

  while (m_clientActive && !m_stopRequested) {
    if (readData() != ReadResult::Datagram) {
      continue;  // a timeout is a pause in the stream; an error cleared m_clientActive
    }
    std::lock_guard<std::mutex> lock(m_dataMutex);
    auto it = m_avatars.find(m_lastAvatarId);
    if (it == m_avatars.end()) {
      continue;
    }
    AvatarStream & av = *it->second;
    // Avatars other than the primary one are discovered mid-stream, so their
    // model is assembled from whichever datagrams have arrived so far.
    buildAvatarModelIncremental(av, m_lastAvatarId);
    if (!av.linksBuilt) {
      continue;
    }
    updateJointAngles(av);
    updateLinkPoses(av);
    updateLinkLinearTwists(av);
    updateLinkAngularTwists(av);
    updateCOM(av);
  }
  m_acquisitionEnded = true;
}

XsensStreamClient::ReadResult XsensStreamClient::readData()
{
  // Read data from UDP socket.  Datagrams belonging to other avatars are
  // skipped and the next one read, so Datagram always means "a datagram for a
  // tracked avatar was parsed" and callers keep their existing contract.
  while (true) {
    auto read_bytes = m_udpSocket->read(m_dataBuffer, MAX_MVN_DATAGRAM_SIZE);
    if (read_bytes > 0) {
      // MVN streams every avatar on the same port: the captured subject plus
      // any tracked objects/props, each tagged with an avatar id.  An object
      // arrives as a one-segment datagram of the very same type as the
      // subject's, so letting it reach the parser would replace the subject's
      // pose data and zero out every segment the object does not define.
      if (read_bytes <= MVN_AVATAR_ID_OFFSET) {
        continue;  // too short to carry an avatar id
      }
      const uint8_t avatar = static_cast<uint8_t>(m_dataBuffer[MVN_AVATAR_ID_OFFSET]);
      if (!m_trackAllAvatars && avatar != m_avatarId) {
        continue;
      }
      const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
      {
        std::lock_guard<std::mutex> lock(m_dataMutex);
        AvatarStream & av = avatarStream(avatar);
        av.parser.readDatagram(m_dataBuffer);
        av.lastDataNs = now_ns;
      }
      m_lastAvatarId = avatar;
      m_lastDataTimeNs = now_ns;
      return ReadResult::Datagram;
    }
    // Receive timeout (EAGAIN/EWOULDBLOCK) is not a real error — the stream may
    // have paused briefly (e.g. during calibration in MVN Studio) or not have
    // started yet.  Callers keep waiting so data is picked up when it resumes.
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return ReadResult::Timeout;
    }
    RCLCPP_ERROR(
      m_logger,
      "Error reading data from UDP socket: %s. Stopping acquisition. "
      "Check: (1) Is MVN Studio still streaming? "
      "(2) Has the network connection been interrupted?",
      strerror(errno));
    m_clientActive = false;
    return ReadResult::Error;
  }
}

bool XsensStreamClient::buildXsensModel()
{
  // The lock is taken only around map/model mutation.  It must never be held
  // across waitFor*Datagram(), because those call readData(), which locks too.
  {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    AvatarStream & av = avatarStream(m_avatarId);
    av.data = std::make_shared<xsens_mvn_ros2::HumanDataHandler>(m_logger);
    av.linkNames.clear();
    av.jointNames.clear();
    av.linksBuilt = false;
    av.jointsBuilt = false;
  }

  auto quaternion_datagram = waitForQuaternionDatagram();
  bool is_object = false;
  {
    std::lock_guard<std::mutex> lock(m_dataMutex);
    AvatarStream & av = avatarStream(m_avatarId);
    if (!buildLinksFromQuaternions(av, quaternion_datagram)) {
      return false;
    }
    is_object = av.isObject;
    if (is_object) {
      RCLCPP_INFO(
        m_logger, "Avatar %u is a rigid object (%zu segment(s)); no joint data expected.",
        static_cast<unsigned>(m_avatarId), av.linkNames.size());
    }
  }
  // A tracked object never sends joint angles, so only a body model waits for
  // them.  Without this an object-only avatar would block for 30 s and then
  // fail with a misleading "no joint elements" error.
  if (is_object) {
    return true;
  }

  auto joint_angles_datagram = waitForJointAnglesDatagram();
  std::lock_guard<std::mutex> lock(m_dataMutex);
  return buildJointsFromAngles(avatarStream(m_avatarId), joint_angles_datagram);
}

void XsensStreamClient::buildAvatarModelIncremental(AvatarStream & av, uint8_t avatar_id)
{
  if (auto * quaternions = av.parser.getQuaternionDatagram()) {
    // dataCount() reads the header, so this costs nothing per frame.
    const size_t segment_count = quaternions->dataCount();

    // MVN can recompose a scene while streaming - an actor joins, avatars get
    // renumbered - and an id can come to mean something entirely different.
    // Without this the first model would be kept forever and applied to the
    // new data: every segment the old list has but the new data lacks would
    // silently read as zero, and the avatar would keep its old body/object
    // classification.
    if (av.linksBuilt && segment_count != av.linkNames.size()) {
      RCLCPP_WARN(
        m_logger,
        "Avatar %u changed from %zu to %zu segment(s); rebuilding its model.",
        static_cast<unsigned>(avatar_id), av.linkNames.size(), segment_count);
      av.data = std::make_shared<xsens_mvn_ros2::HumanDataHandler>(m_logger);
      av.linkNames.clear();
      av.jointNames.clear();
      av.linksBuilt = false;
      av.jointsBuilt = false;
    }

    if (!av.linksBuilt) {
      if (buildLinksFromQuaternions(av, *quaternions)) {
        RCLCPP_INFO(
          m_logger, "Avatar %u: %zu segment(s)%s.",
          static_cast<unsigned>(avatar_id), av.linkNames.size(),
          av.isObject ? " (rigid object)" : " (body)");
      }
      return;
    }
  }

  if (!av.linksBuilt) {
    return;
  }
  // A body model's joints arrive in their own datagram, which may be parsed
  // several frames after the first quaternion datagram.
  if (!av.isObject && !av.jointsBuilt) {
    if (auto * joint_angles = av.parser.getJointAnglesDatagram()) {
      buildJointsFromAngles(av, *joint_angles);
    }
  }
}

bool XsensStreamClient::buildLinksFromQuaternions(
  AvatarStream & av, QuaternionDatagram & quaternions)
{
  XsensModelNames xsens_model_names;

  // Segment IDs 1..23 are the standard body model, indexed
  // directly into xsens_model_names.links
  // Any segments beyond that are assumed to be MANUS finger-tracking data,
  // appended by MVN as [left hand block][right hand block]
  // Each hand block is 20 segments (5 fingers x 4 phalanges)
  constexpr int kBodySegmentCount = 23;

  const auto segments = quaternions.getData();
  const int total_segments = static_cast<int>(segments.size());
  if (total_segments == 0) {
    RCLCPP_ERROR(m_logger, "No link elements found in quaternion datagram.");
    return false;
  }

  av.linkNames.clear();

  // Anything smaller than the body model is a tracked object (a prop), which
  // MVN streams as a rigid body: a handful of segments and no joints.
  av.isObject = total_segments < kBodySegmentCount;
  if (av.isObject) {
    for (int i = 0; i < total_segments; ++i) {
      // "base_link" has no kinetic parent, so SkeletonPublisher broadcasts it
      // as an absolute pose in the reference frame - what a rigid body needs.
      const std::string link_name = (i == 0) ? "base_link" : "link_" + std::to_string(i + 1);
      if (!av.data->setLink(link_name, xsens_mvn_ros2::Link(link_name))) {
        RCLCPP_ERROR(m_logger, "Error inserting link '%s'. Aborting model build.",
          link_name.c_str());
        return false;
      }
      av.linkNames.push_back(link_name);
    }
    av.linksBuilt = true;
    return true;
  }

  RCLCPP_INFO(m_logger, "Building model: %d links available.", total_segments);

  const int extra_segments = std::max(0, total_segments - kBodySegmentCount);
  const int segments_per_hand = extra_segments / 2;
  if (extra_segments % 2 != 0) {
    RCLCPP_WARN(
      m_logger,
      "Received %d segments beyond the standard %d-segment body model (odd number) "
      "so the left/right hand split below is likely wrong.",
      extra_segments, kBodySegmentCount);
  }
  if (segments_per_hand > 0) {
    RCLCPP_INFO(
      m_logger, "MANUS gloves finger tracking data detected: %d segments per hand.",
      segments_per_hand);
  }

  for (const auto & xsens_link : segments) {
    const int segment_id = xsens_link.segmentId;
    std::string link_name;

    if (segment_id >= 1 && segment_id <= kBodySegmentCount) {
      link_name = xsens_model_names.links[segment_id];
    } else if (  // NOLINT(readability/braces)
      segments_per_hand > 0 && segment_id > kBodySegmentCount &&
      segment_id <= kBodySegmentCount + 2 * segments_per_hand)
    {
      const int offset = segment_id - kBodySegmentCount - 1;  // 0-based, both hands
      const bool is_left = offset < segments_per_hand;
      const int position = is_left ? offset : offset - segments_per_hand;
      link_name = fingerSegmentName(is_left ? "left" : "right", position);
    } else {
      RCLCPP_WARN(
        m_logger,
        "Segment ID %d is outside the known body/finger model range "
        "(%d total segments this frame); skipping.",
        segment_id, total_segments);
      continue;
    }

    if (!av.data->setLink(link_name, xsens_mvn_ros2::Link(link_name))) {
      RCLCPP_ERROR(m_logger, "Error inserting link '%s'. Aborting model build.",
        link_name.c_str());
      return false;
    }
    av.linkNames.push_back(link_name);
  }

  if (av.data->getLinks().empty()) {
    RCLCPP_ERROR(m_logger, "No link elements found in quaternion datagram.");
    return false;
  }
  av.linksBuilt = true;
  return true;
}

bool XsensStreamClient::buildJointsFromAngles(
  AvatarStream & av, JointAnglesDatagram & joint_angles)
{
  XsensModelNames xsens_model_names;
  const auto joints = joint_angles.getData();

  RCLCPP_INFO(m_logger, "Building model: %zu joints available.", joints.size());

  for (size_t joint_cnt = 0; joint_cnt < joints.size(); joint_cnt++) {
    // The name table covers the joints MVN is known to send.  Anything beyond
    // it would index out of bounds, so stop rather than read past the end.
    if (joint_cnt >= xsens_model_names.joints.size()) {
      RCLCPP_WARN(
        m_logger,
        "Received %zu joints but only %zu are named in the model table; ignoring the rest.",
        joints.size(), xsens_model_names.joints.size());
      break;
    }
    const auto & joint_name = xsens_model_names.joints[joint_cnt];
    if (!av.data->setJoint(joint_name, xsens_mvn_ros2::Joint(joint_name))) {
      RCLCPP_ERROR(m_logger, "Error inserting joint '%s'. Aborting model build.",
        joint_name.c_str());
      return false;
    }
    av.jointNames.push_back(joint_name);
  }

  if (av.data->getJoints().empty()) {
    RCLCPP_ERROR(m_logger, "No joint elements found in joint angles datagram.");
    return false;
  }
  av.jointsBuilt = true;
  return true;
}

QuaternionDatagram XsensStreamClient::waitForQuaternionDatagram()
{
  RCLCPP_INFO(m_logger, "Waiting for quaternion datagram...");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (true) {
    {
      std::lock_guard<std::mutex> lock(m_dataMutex);
      if (auto * datagram = avatarStream(m_avatarId).parser.getQuaternionDatagram()) {
        RCLCPP_INFO(m_logger, "Quaternion datagram received.");
        return *datagram;
      }
    }
    if (m_stopRequested) {
      return QuaternionDatagram();
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        m_logger,
        "Timeout waiting for quaternion datagram after 30 seconds. "
        "Check: (1) Is MVN Studio streaming? "
        "(2) Is the quaternion datagram enabled in MVN Studio network settings? "
        "(3) Is the correct UDP port (%d) configured?",
        m_udpPort);
      return QuaternionDatagram();
    }
    // A receive timeout just means nothing arrived in the last second; keep
    // waiting until the deadline.  Only a socket error ends the wait early.
    if (readData() == ReadResult::Error) {
      return QuaternionDatagram();
    }
  }
}

JointAnglesDatagram XsensStreamClient::waitForJointAnglesDatagram()
{
  RCLCPP_INFO(m_logger, "Waiting for joint angles datagram...");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (true) {
    {
      std::lock_guard<std::mutex> lock(m_dataMutex);
      if (auto * datagram = avatarStream(m_avatarId).parser.getJointAnglesDatagram()) {
        RCLCPP_INFO(m_logger, "Joint angles datagram received.");
        return *datagram;
      }
    }
    if (m_stopRequested) {
      return JointAnglesDatagram();
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(
        m_logger,
        "Timeout waiting for joint angles datagram after 30 s. "
        "Check: (1) Is MVN Studio streaming? "
        "(2) Is the joint angles datagram enabled in "
        "MVN Studio network settings? "
        "(3) Is the correct UDP port (%d) configured?",
        m_udpPort);
      return JointAnglesDatagram();
    }
    if (readData() == ReadResult::Error) {
      return JointAnglesDatagram();
    }
  }
}

void XsensStreamClient::updateJointAngles(AvatarStream & av)
{
  if (av.isObject || !av.jointsBuilt) {
    return;  // a rigid object has no joints to update
  }
  auto joint_angles = av.parser.getJointAnglesDatagram();
  if (joint_angles) {
    // Update joint angles according to N-pose
    av.data->setJointAngles(
      "l5_s1", jointAngleToEigenVector3d(joint_angles->getItem(1, 2), 1, 1, 1));
    av.data->setJointAngles(
      "l4_l3", jointAngleToEigenVector3d(joint_angles->getItem(2, 3), 1, 1, 1));
    av.data->setJointAngles(
      "l1_t12", jointAngleToEigenVector3d(joint_angles->getItem(3, 4), 1, 1, 1));
    av.data->setJointAngles(
      "t9_t8", jointAngleToEigenVector3d(joint_angles->getItem(4, 5), 1, 1, 1));
    av.data->setJointAngles(
      "t1_c7", jointAngleToEigenVector3d(joint_angles->getItem(5, 6), 1, 1, 1));
    av.data->setJointAngles(
      "c1_head", jointAngleToEigenVector3d(joint_angles->getItem(6, 7), 1, 1, 1));
    av.data->setJointAngles(
      "right_c7_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(5, 8), -1, 1, -1));
    av.data->setJointAngles(
      "right_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(8, 9), -1, 1, -1));
    av.data->setJointAngles(
      "right_elbow", jointAngleToEigenVector3d(joint_angles->getItem(9, 10), -1, 1, -1));
    av.data->setJointAngles(
      "right_wrist", jointAngleToEigenVector3d(joint_angles->getItem(10, 11), -1, 1, -1));
    av.data->setJointAngles(
      "left_c7_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(5, 12), 1, -1, -1));
    av.data->setJointAngles(
      "left_shoulder", jointAngleToEigenVector3d(joint_angles->getItem(12, 13), 1, -1, -1));
    av.data->setJointAngles(
      "left_elbow", jointAngleToEigenVector3d(joint_angles->getItem(13, 14), 1, -1, -1));
    av.data->setJointAngles(
      "left_wrist", jointAngleToEigenVector3d(joint_angles->getItem(14, 15), 1, -1, -1));
    av.data->setJointAngles(
      "right_hip", jointAngleToEigenVector3d(joint_angles->getItem(1, 16), -1, 1, -1));
    av.data->setJointAngles(
      "right_knee", jointAngleToEigenVector3d(joint_angles->getItem(16, 17), -1, 1, 1));
    av.data->setJointAngles(
      "right_ankle", jointAngleToEigenVector3d(joint_angles->getItem(17, 18), -1, 1, -1));
    av.data->setJointAngles(
      "right_ballfoot", jointAngleToEigenVector3d(joint_angles->getItem(18, 19), -1, 1, -1));
    av.data->setJointAngles(
      "left_hip", jointAngleToEigenVector3d(joint_angles->getItem(1, 20), 1, -1, -1));
    av.data->setJointAngles(
      "left_knee", jointAngleToEigenVector3d(joint_angles->getItem(20, 21), 1, -1, 1));
    av.data->setJointAngles(
      "left_ankle", jointAngleToEigenVector3d(joint_angles->getItem(21, 22), 1, -1, -1));
    av.data->setJointAngles(
      "left_ballfoot", jointAngleToEigenVector3d(joint_angles->getItem(22, 23), 1, -1, -1));

    // Add +90° to elbows angle
    xsens_mvn_ros2::Joint elbow_joint;
    av.data->getJoint("right_elbow", elbow_joint);
    elbow_joint.state.angles[2] -= 90.0;
    av.data->setJoint("right_elbow", elbow_joint);
    av.data->getJoint("left_elbow", elbow_joint);
    elbow_joint.state.angles[2] += 90.0;
    av.data->setJoint("left_elbow", elbow_joint);
  }
}

void XsensStreamClient::updateLinkPoses(AvatarStream & av)
{
  auto quaternion_datagram = av.parser.getQuaternionDatagram();
  if (!quaternion_datagram) {
    return;
  }
  for (size_t link_cnt = 0; link_cnt < av.linkNames.size(); link_cnt++) {
    auto link_pose = quaternion_datagram->getItem(static_cast<int32_t>(link_cnt) + 1);
    Eigen::Vector3d link_pos;
    link_pos << link_pose.sensorPos[0], link_pose.sensorPos[1], link_pose.sensorPos[2];
    Eigen::Quaterniond link_orient(
      link_pose.quatRotation[0], link_pose.quatRotation[1], link_pose.quatRotation[2],
      link_pose.quatRotation[3]);
    av.data->setLinkPose(av.linkNames[link_cnt], link_pos, link_orient);
  }

  if (av.isObject) {
    return;  // no arms to correct on a rigid object
  }

  Eigen::Quaterniond quat_x_rot_90neg(0.7071068, -0.7071068, 0.0, 0.0);
  rotateLink(av, "right_upper_arm", quat_x_rot_90neg);
  rotateLink(av, "right_forearm", quat_x_rot_90neg);
  rotateLink(av, "right_hand", quat_x_rot_90neg);

  Eigen::Quaterniond quat_x_rot_90pos(0.7071068, 0.7071068, 0.0, 0.0);
  rotateLink(av, "left_upper_arm", quat_x_rot_90pos);
  rotateLink(av, "left_forearm", quat_x_rot_90pos);
  rotateLink(av, "left_hand", quat_x_rot_90pos);
}

void XsensStreamClient::rotateLink(
  AvatarStream & av, const std::string & link_name,
  const Eigen::Quaterniond & quat) const
{
  xsens_mvn_ros2::Link link_to_rotate;
  if (!av.data->getLink(link_name, link_to_rotate)) {
    RCLCPP_WARN_ONCE(m_logger, "Link '%s' not found for rotation correction.", link_name.c_str());
    return;
  }
  link_to_rotate.state.orientation = link_to_rotate.state.orientation * quat;
  av.data->setLink(link_name, link_to_rotate);
}

void XsensStreamClient::updateLinkLinearTwists(AvatarStream & av)
{
  auto linear_segment_kinematics_datagram = av.parser.getLinearSegmentKinematicsDatagram();
  if (!linear_segment_kinematics_datagram) {
    return;
  }
  for (size_t link_cnt = 0; link_cnt < av.linkNames.size(); link_cnt++) {
    // Get linear kinematics from datagram structure
    auto link_linear_kinematics =
      linear_segment_kinematics_datagram->getItem(static_cast<int32_t>(link_cnt) + 1);

    // Get link
    xsens_mvn_ros2::Link link;
    if (!av.data->getLink(av.linkNames[link_cnt], link)) {
      RCLCPP_WARN_ONCE(m_logger, "Link '%s' not found in human data handler (linear twists).",
        av.linkNames[link_cnt].c_str());
    } else {
      link.state.velocity.linear << link_linear_kinematics.velocity[0],
        link_linear_kinematics.velocity[1], link_linear_kinematics.velocity[2];
      link.state.acceleration.linear << link_linear_kinematics.acceleration[0],
        link_linear_kinematics.acceleration[1], link_linear_kinematics.acceleration[2];

      av.data->setLinkState(av.linkNames[link_cnt], link.state);
    }
  }
}

void XsensStreamClient::updateLinkAngularTwists(AvatarStream & av)
{
  auto angular_segment_kinematics_datagram = av.parser.getAngularSegmentKinematicsDatagram();
  if (!angular_segment_kinematics_datagram) {
    return;
  }
  for (size_t link_cnt = 0; link_cnt < av.linkNames.size(); link_cnt++) {
    // Get angular kinematics from datagram structure
    auto link_angular_kinematics =
      angular_segment_kinematics_datagram->getItem(static_cast<int32_t>(link_cnt) + 1);
    // Get link
    xsens_mvn_ros2::Link link;
    if (!av.data->getLink(av.linkNames[link_cnt], link)) {
      RCLCPP_WARN_ONCE(m_logger, "Link '%s' not found in human data handler (angular twists).",
        av.linkNames[link_cnt].c_str());
    } else {
      link.state.velocity.angular << link_angular_kinematics.angularVeloc[0] * M_PI / 180,
        link_angular_kinematics.angularVeloc[1] * M_PI / 180,
        link_angular_kinematics.angularVeloc[2] * M_PI / 180;
      link.state.acceleration.angular << link_angular_kinematics.angularAccel[0] * M_PI / 180,
        link_angular_kinematics.angularAccel[1] * M_PI / 180,
        link_angular_kinematics.angularAccel[2] * M_PI / 180;
      av.data->setLinkState(av.linkNames[link_cnt], link.state);
    }
  }
}

void XsensStreamClient::updateCOM(AvatarStream & av)
{
  auto com_datagram = av.parser.getCenterOfMassDatagram();
  if (!com_datagram) {
    return;
  }
  Eigen::Vector3d com;
  com << com_datagram->getData()[0], com_datagram->getData()[1], com_datagram->getData()[2];
  av.data->setCOM(com);
}

Eigen::Vector3d XsensStreamClient::jointAngleToEigenVector3d(
  const JointAngle & joint_angle, const double & x_axis, const double & y_axis,
  const double & z_axis) const
{
  // Convert JointAngle object to a Eigen Vector3d rotating axes to align in resting position N-Pose
  Eigen::Vector3d joint_angle_eigen_vec;
  joint_angle_eigen_vec[0] = x_axis * joint_angle.rotation[0];
  // TEMPORARY HORRIBLE SOLUTION
  joint_angle_eigen_vec[1] = z_axis * joint_angle.rotation[2];
  joint_angle_eigen_vec[2] = y_axis * joint_angle.rotation[1];
  return joint_angle_eigen_vec;
}

std::unordered_map<std::string,
  xsens_mvn_ros2::SegmentKinematics> XsensStreamClient::getAvatarSegments(uint8_t avatar_id) const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  std::unordered_map<std::string, xsens_mvn_ros2::SegmentKinematics> segments;
  const AvatarStream * av = findAvatar(avatar_id);
  if (!av || !av->data || !av->linksBuilt) {
    return segments;
  }
  for (const auto & [name, link] : av->data->getLinks()) {
    xsens_mvn_ros2::SegmentKinematics kin;
    kin.position = link.state.position;
    kin.orientation = link.state.orientation;
    kin.velocity_linear = link.state.velocity.linear;
    kin.velocity_angular = link.state.velocity.angular;
    kin.accel_linear = link.state.acceleration.linear;
    kin.accel_angular = link.state.acceleration.angular;
    segments[name] = kin;
  }
  return segments;
}

std::vector<xsens_mvn_ros2::JointAngles> XsensStreamClient::getAvatarJoints(uint8_t avatar_id) const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  std::vector<xsens_mvn_ros2::JointAngles> joint_vec;
  const AvatarStream * av = findAvatar(avatar_id);
  if (!av || !av->data) {
    return joint_vec;
  }
  for (const auto & [name, joint] : av->data->getJoints()) {
    joint_vec.push_back(
      {name, joint.state.angles[0] * M_PI / 180.0, joint.state.angles[1] * M_PI / 180.0,
        joint.state.angles[2] * M_PI / 180.0});
  }
  return joint_vec;
}

std::optional<Eigen::Vector3d> XsensStreamClient::getAvatarCOM(uint8_t avatar_id) const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  const AvatarStream * av = findAvatar(avatar_id);
  if (!av || !av->data) {
    return std::nullopt;
  }
  return av->data->getCOM();
}

int64_t XsensStreamClient::avatarLastDataTimeNs(uint8_t avatar_id) const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  const AvatarStream * av = findAvatar(avatar_id);
  return av ? av->lastDataNs.load() : 0;
}

bool XsensStreamClient::isObject(uint8_t avatar_id) const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  const AvatarStream * av = findAvatar(avatar_id);
  return av && av->isObject;
}

std::vector<uint8_t> XsensStreamClient::avatarIds() const
{
  std::lock_guard<std::mutex> lock(m_dataMutex);
  std::vector<uint8_t> ids;
  ids.reserve(m_avatars.size());
  for (const auto & [id, av] : m_avatars) {
    if (av->linksBuilt) {
      ids.push_back(id);
    }
  }
  return ids;
}

// ---- IMotionCaptureSource: the primary avatar ----

std::unordered_map<std::string,
  xsens_mvn_ros2::SegmentKinematics> XsensStreamClient::getSegments() const
{
  return getAvatarSegments(m_avatarId);
}

std::vector<xsens_mvn_ros2::JointAngles> XsensStreamClient::getJoints() const
{
  return getAvatarJoints(m_avatarId);
}

std::optional<Eigen::Vector3d> XsensStreamClient::getCOM() const
{
  return getAvatarCOM(m_avatarId);
}

XsensStreamClient::~XsensStreamClient()
{
  // The thread may still be waiting for its first datagram; the stop flag
  // makes that wait return at the next 1 s receive timeout.
  m_stopRequested = true;
  m_clientActive = false;
  if (m_dataAcquisitionThread.joinable()) {
    m_dataAcquisitionThread.join();
  }
  RCLCPP_DEBUG(m_logger, "XsensStreamClient destroyed.");
}
