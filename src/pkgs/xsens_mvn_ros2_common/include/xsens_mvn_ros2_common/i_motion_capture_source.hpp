// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <xsens_mvn_ros2_common/skeleton_publisher.hpp>

namespace xsens_mvn_ros2
{

/// Abstract interface for a motion capture data source.
/// Both XsensStreamClient (UDP) and future sources implement this interface,
/// enabling unit tests to inject mock data sources without network/hardware.
class IMotionCaptureSource
{
public:
  virtual ~IMotionCaptureSource() = default;

  /// Returns a snapshot of the current segment kinematics, keyed by canonical segment name.
  /// Thread-safe: must be safe to call from a timer callback.
  virtual std::unordered_map<std::string, SegmentKinematics> getSegments() const = 0;

  /// Returns a snapshot of the current joint angles (in radians).
  virtual std::vector<JointAngles> getJoints() const = 0;

  /// Returns the center of mass position in world frame, or nullopt if unavailable.
  virtual std::optional<Eigen::Vector3d> getCOM() const = 0;

  /// Returns nanoseconds (steady clock) of the last data arrival, or 0 if no data yet.
  virtual int64_t lastDataTimeNs() const = 0;

  /// Returns true if the source is actively receiving data.
  virtual bool isActive() const = 0;

  // ---- Multi-avatar access ----
  //
  // MVN can stream several avatars at once (multiple suits, plus tracked
  // objects).  Sources that support it override these; the defaults describe a
  // single-avatar source, so existing implementations and mocks keep working.

  /// Ids of every avatar this source can report.
  virtual std::vector<uint8_t> avatarIds() const {return {0};}

  // Deliberately NOT overloads of the single-avatar getters above: a derived
  // class that overrides getSegments() would hide a getSegments(uint8_t)
  // overload, silently breaking multi-avatar access through that class.

  virtual std::unordered_map<std::string, SegmentKinematics> getAvatarSegments(uint8_t) const
  {
    return getSegments();
  }

  virtual std::vector<JointAngles> getAvatarJoints(uint8_t) const {return getJoints();}

  virtual std::optional<Eigen::Vector3d> getAvatarCOM(uint8_t) const {return getCOM();}

  virtual int64_t avatarLastDataTimeNs(uint8_t) const {return lastDataTimeNs();}

  /// True if the avatar is a rigid object (a tracked prop) rather than a body.
  virtual bool isObject(uint8_t) const {return false;}
};

}  // namespace xsens_mvn_ros2
