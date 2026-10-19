// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include <xsens_mvn_ros2_description/segment_defs.hpp>

using xsens_mvn_ros2_description::kFingerSegments;
using xsens_mvn_ros2_description::kSegments;
using xsens_mvn_ros2_description::ScaleAxis;
using xsens_mvn_ros2_description::SegmentDef;

/// Body segments plus both finger blocks — the largest tree the publisher can emit.
std::vector<SegmentDef> allSegments()
{
  std::vector<SegmentDef> all(kSegments.begin(), kSegments.end());
  all.insert(all.end(), kFingerSegments.begin(), kFingerSegments.end());
  return all;
}

TEST(SegmentDefs, NoNullParentFields)
{
  for (const auto & seg : allSegments()) {
    EXPECT_NE(seg.parent, nullptr) << "Segment " << seg.name << " has null parent";
    EXPECT_NE(std::string(seg.parent), "") << "Segment " << seg.name << " has empty parent "
                                           << "(should be 'world' for root)";
  }
}

TEST(SegmentDefs, NoduplicateNames)
{
  std::unordered_set<std::string> names;
  for (const auto & seg : allSegments()) {
    EXPECT_TRUE(names.insert(std::string(seg.name)).second)
      << "Duplicate segment name: " << seg.name;
  }
}

TEST(SegmentDefs, NonLeafRefLengthsPositive)
{
  for (const auto & seg : allSegments()) {
    if (std::string(seg.child).empty()) {
      continue;  // leaf
    }
    if (std::string(seg.mesh).empty()) {
      continue;  // no mesh shipped, so there is nothing to scale
    }
    EXPECT_GT(seg.refLength, 0.0) << "Non-leaf segment " << seg.name << " has refLength <= 0";
  }
}

TEST(SegmentDefs, AllParentsExistOrWorld)
{
  std::unordered_set<std::string> names;
  for (const auto & seg : allSegments()) {
    names.insert(std::string(seg.name));
  }
  for (const auto & seg : allSegments()) {
    const std::string parent(seg.parent);
    if (parent == "world") {
      continue;
    }
    EXPECT_NE(names.find(parent), names.end())
      << "Segment " << seg.name << " references unknown parent: " << parent;
  }
}

// --- MANUS finger block ---

TEST(FingerSegments, TwentySegmentsPerHand)
{
  int left = 0, right = 0;
  for (const auto & seg : kFingerSegments) {
    const std::string n(seg.name);
    if (n.rfind("left_", 0) == 0) {
      left++;
    } else if (n.rfind("right_", 0) == 0) {
      right++;
    } else {
      ADD_FAILURE() << "Unexpected finger segment name: " << n;
    }
  }
  // MVN streams each hand as one 20-segment block.
  EXPECT_EQ(left, 20);
  EXPECT_EQ(right, 20);
}

TEST(FingerSegments, NamesMatchTheMvnHandModel)
{
  // The names are MVN's own, transliterated to snake_case, and in the order
  // MVN streams them.  The thumb ("first") has no middle phalange.
  const std::vector<std::string> expected = {
    "carpus",
    "first_mc", "first_pp", "first_dp",
    "second_mc", "second_pp", "second_mp", "second_dp",
    "third_mc", "third_pp", "third_mp", "third_dp",
    "fourth_mc", "fourth_pp", "fourth_mp", "fourth_dp",
    "fifth_mc", "fifth_pp", "fifth_mp", "fifth_dp"};

  for (const auto & side : {std::string("left"), std::string("right")}) {
    for (size_t i = 0; i < expected.size(); ++i) {
      const std::string name = side + "_" + expected[i];
      const auto it = std::find_if(
        kFingerSegments.begin(), kFingerSegments.end(),
        [&](const SegmentDef & s) {return std::string(s.name) == name;});
      EXPECT_NE(it, kFingerSegments.end()) << "Missing finger segment " << name;
    }
  }
}

TEST(FingerSegments, ChainsFollowTheHandModel)
{
  const std::vector<std::string> order = {
    "carpus",
    "first_mc", "first_pp", "first_dp",
    "second_mc", "second_pp", "second_mp", "second_dp",
    "third_mc", "third_pp", "third_mp", "third_dp",
    "fourth_mc", "fourth_pp", "fourth_mp", "fourth_dp",
    "fifth_mc", "fifth_pp", "fifth_mp", "fifth_dp"};

  for (const auto & side : {std::string("left"), std::string("right")}) {
    for (size_t i = 0; i < order.size(); ++i) {
      const std::string name = side + "_" + order[i];
      const auto it = std::find_if(
        kFingerSegments.begin(), kFingerSegments.end(),
        [&](const SegmentDef & s) {return std::string(s.name) == name;});
      ASSERT_NE(it, kFingerSegments.end()) << "Missing " << name;

      // Carpus hangs off the hand, as does each finger's metacarpal; every
      // other bone hangs off the previous one in its own finger.
      std::string expectedParent;
      if (i == 0) {
        expectedParent = side + "_hand";
      } else if (i == 1) {
        expectedParent = side + "_carpus";
      } else if (i % 4 == 0) {
        expectedParent = side + "_hand";
      } else {
        expectedParent = side + "_" + order[i - 1];
      }
      EXPECT_EQ(std::string(it->parent), expectedParent) << "for " << name;

      // Distal phalanges end each finger, so they are leaves.
      const bool isLeaf = (i == 3 || i == 7 || i == 11 || i == 15 || i == 19);
      const std::string expectedChild = isLeaf ? "" : side + "_" + order[i + 1];
      EXPECT_EQ(std::string(it->child), expectedChild) << "for " << name;
    }
  }
}

TEST(FingerSegments, MeshesAreSideCorrect)
{
  for (const auto & seg : kFingerSegments) {
    const std::string mesh(seg.mesh);
    if (mesh.empty()) {
      continue;  // SecondMC / FifthMC are not shipped
    }
    const std::string expected = (std::string(seg.name).rfind("left", 0) == 0) ? "Left" : "Right";
    EXPECT_EQ(mesh.rfind(expected, 0), 0u)
      << "Segment " << seg.name << " uses mesh " << mesh << " from the wrong hand";
  }
}

TEST(FingerSegments, VisualAnchorMatchesHandedness)
{
  // Finger meshes share an origin at y = -/+0.0706 m; the visual offset moves
  // each bone's proximal end onto its joint, mirrored per hand.
  for (const auto & seg : kFingerSegments) {
    const bool isLeft = std::string(seg.name).rfind("left", 0) == 0;
    EXPECT_EQ(std::string(seg.visualXyz), isLeft ? "0 0.0706 0" : "0 -0.0706 0")
      << "for " << seg.name;
    EXPECT_EQ(std::string(seg.visualRpy), "0 0 0") << "for " << seg.name;
    EXPECT_EQ(seg.scaleAxis, ScaleAxis::Y) << "for " << seg.name;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
