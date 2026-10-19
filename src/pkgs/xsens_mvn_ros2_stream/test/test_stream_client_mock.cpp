// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <xsens_mvn_ros2_common/i_motion_capture_source.hpp>

namespace
{

/// Minimal mock that returns canned data for testing consumers of IMotionCaptureSource.
class MockMotionCaptureSource : public xsens_mvn_ros2::IMotionCaptureSource
{
public:
  std::unordered_map<std::string, xsens_mvn_ros2::SegmentKinematics> getSegments() const override
  {
    xsens_mvn_ros2::SegmentKinematics kin;
    kin.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    kin.orientation = Eigen::Quaterniond::Identity();
    kin.velocity_linear = Eigen::Vector3d::Zero();
    kin.velocity_angular = Eigen::Vector3d::Zero();
    kin.accel_linear = Eigen::Vector3d::Zero();
    kin.accel_angular = Eigen::Vector3d::Zero();
    return {{"pelvis", kin}};
  }

  std::vector<xsens_mvn_ros2::JointAngles> getJoints() const override
  {
    return {{"l5_s1", 0.1, 0.2, 0.3}};
  }

  std::optional<Eigen::Vector3d> getCOM() const override {return Eigen::Vector3d(0.5, 0.0, 1.0);}

  int64_t lastDataTimeNs() const override {return 42000000LL;}
  bool isActive() const override {return true;}
};

}  // namespace

TEST(MockMotionCaptureSource, GetSegmentsReturnsCannedPelvis)
{
  MockMotionCaptureSource mock;
  auto segments = mock.getSegments();
  ASSERT_EQ(segments.size(), 1u);
  ASSERT_TRUE(segments.count("pelvis"));
  EXPECT_DOUBLE_EQ(segments["pelvis"].position.x(), 1.0);
  EXPECT_DOUBLE_EQ(segments["pelvis"].position.y(), 2.0);
  EXPECT_DOUBLE_EQ(segments["pelvis"].position.z(), 3.0);
}

TEST(MockMotionCaptureSource, GetJointsReturnsCannedJoint)
{
  MockMotionCaptureSource mock;
  auto joints = mock.getJoints();
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].name, "l5_s1");
  EXPECT_DOUBLE_EQ(joints[0].x, 0.1);
  EXPECT_DOUBLE_EQ(joints[0].y, 0.2);
  EXPECT_DOUBLE_EQ(joints[0].z, 0.3);
}

TEST(MockMotionCaptureSource, GetCOMReturnsCannedValue)
{
  MockMotionCaptureSource mock;
  auto com = mock.getCOM();
  ASSERT_TRUE(com.has_value());
  EXPECT_DOUBLE_EQ(com->x(), 0.5);
  EXPECT_DOUBLE_EQ(com->z(), 1.0);
}

TEST(MockMotionCaptureSource, IsActiveReturnsTrue)
{
  MockMotionCaptureSource mock;
  EXPECT_TRUE(mock.isActive());
}

TEST(MockMotionCaptureSource, LastDataTimeNsReturnsExpected)
{
  MockMotionCaptureSource mock;
  EXPECT_EQ(mock.lastDataTimeNs(), 42000000LL);
}

// ---- Multi-avatar defaults on IMotionCaptureSource ----
//
// MVN streams several avatars at once, but a single-avatar source (like the
// XME node, or any existing mock) must keep working untouched.  These pin the
// back-compatible defaults the interface promises.

TEST(MockMotionCaptureSource, DefaultsToASingleAvatarZero)
{
  MockMotionCaptureSource mock;
  EXPECT_EQ(mock.avatarIds(), std::vector<uint8_t>{0});
}

TEST(MockMotionCaptureSource, PerAvatarAccessorsFallBackToTheSingleAvatar)
{
  MockMotionCaptureSource mock;
  // A source that does not implement multi-avatar reports the same data for
  // any id, matching its single-avatar accessors.
  EXPECT_EQ(mock.getAvatarSegments(0).size(), mock.getSegments().size());
  EXPECT_EQ(mock.getAvatarJoints(0).size(), mock.getJoints().size());
  ASSERT_TRUE(mock.getAvatarCOM(0).has_value());
  EXPECT_TRUE(mock.getAvatarCOM(0)->isApprox(*mock.getCOM()));
  EXPECT_EQ(mock.avatarLastDataTimeNs(0), mock.lastDataTimeNs());
}

TEST(MockMotionCaptureSource, NothingIsAnObjectByDefault)
{
  MockMotionCaptureSource mock;
  EXPECT_FALSE(mock.isObject(0));
  EXPECT_FALSE(mock.isObject(7));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
