// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// Maps XME SDK segment names (PascalCase, or SDK-specific labels like "Sternum")
// to the canonical snake_case names used as TF frame suffixes by both nodes.
inline std::string xmeSegmentToCanonical(const std::string & sdk_name)
{
  static const std::unordered_map<std::string, std::string> mapping = {
    {"Pelvis", "pelvis"},
    {"L5", "l5"},
    {"L3", "l3"},
    {"T12", "t12"},
    {"T8", "t8"},
    {"Sternum", "t8"},  // Some XME SDK versions label the T8 tracker segment "Sternum"
    {"Neck", "neck"},
    {"Head", "head"},
    {"RightShoulder", "right_shoulder"},
    {"RightUpperArm", "right_upper_arm"},
    {"RightForeArm", "right_forearm"},
    {"RightHand", "right_hand"},
    {"LeftShoulder", "left_shoulder"},
    {"LeftUpperArm", "left_upper_arm"},
    {"LeftForeArm", "left_forearm"},
    {"LeftHand", "left_hand"},
    {"RightUpperLeg", "right_upper_leg"},
    {"RightLowerLeg", "right_lower_leg"},
    {"RightFoot", "right_foot"},
    {"RightToe", "right_toe"},
    {"LeftUpperLeg", "left_upper_leg"},
    {"LeftLowerLeg", "left_lower_leg"},
    {"LeftFoot", "left_foot"},
    {"LeftToe", "left_toe"},
    // Finger segments (MANUS gloves); see handSegmentSuffixes() below.
    {"LeftCarpus", "left_carpus"}, {"RightCarpus", "right_carpus"},
    {"LeftFirstMC", "left_first_mc"}, {"LeftFirstPP", "left_first_pp"},
    {"LeftFirstDP", "left_first_dp"},
    {"LeftSecondMC", "left_second_mc"}, {"LeftSecondPP", "left_second_pp"},
    {"LeftSecondMP", "left_second_mp"}, {"LeftSecondDP", "left_second_dp"},
    {"LeftThirdMC", "left_third_mc"}, {"LeftThirdPP", "left_third_pp"},
    {"LeftThirdMP", "left_third_mp"}, {"LeftThirdDP", "left_third_dp"},
    {"LeftFourthMC", "left_fourth_mc"}, {"LeftFourthPP", "left_fourth_pp"},
    {"LeftFourthMP", "left_fourth_mp"}, {"LeftFourthDP", "left_fourth_dp"},
    {"LeftFifthMC", "left_fifth_mc"}, {"LeftFifthPP", "left_fifth_pp"},
    {"LeftFifthMP", "left_fifth_mp"}, {"LeftFifthDP", "left_fifth_dp"},
    {"RightFirstMC", "right_first_mc"}, {"RightFirstPP", "right_first_pp"},
    {"RightFirstDP", "right_first_dp"},
    {"RightSecondMC", "right_second_mc"}, {"RightSecondPP", "right_second_pp"},
    {"RightSecondMP", "right_second_mp"}, {"RightSecondDP", "right_second_dp"},
    {"RightThirdMC", "right_third_mc"}, {"RightThirdPP", "right_third_pp"},
    {"RightThirdMP", "right_third_mp"}, {"RightThirdDP", "right_third_dp"},
    {"RightFourthMC", "right_fourth_mc"}, {"RightFourthPP", "right_fourth_pp"},
    {"RightFourthMP", "right_fourth_mp"}, {"RightFourthDP", "right_fourth_dp"},
    {"RightFifthMC", "right_fifth_mc"}, {"RightFifthPP", "right_fifth_pp"},
    {"RightFifthMP", "right_fifth_mp"}, {"RightFifthDP", "right_fifth_dp"},
  };
  const auto it = mapping.find(sdk_name);
  return (it != mapping.end()) ? it->second : sdk_name;
}

// Finger-tracking segment naming (MANUS gloves via MVN)
//
// When finger tracking is enabled we get extra segments in addition to the
// standard 23 body segments.  A hand's block is 20 segments, and it is NOT a
// uniform 5 x 4 grid of phalanges - it is the MVN hand model, which happens to
// also total 20:
//
//   carpus (1) + thumb MC/PP/DP (3) + four fingers x MC/PP/MP/DP (16)
//
// The names below mirror MVN's own (LeftCarpus, LeftFirstMC, LeftSecondPP, ...)
// transliterated to canonical snake_case the same way the body segments are,
// so a TF frame reads e.g. "skeleton_left_first_mc".  Verified against the
// segment names MVN reports in its scaling datagram and against live TF bone
// lengths.
//
// Note the thumb ("first") has no middle phalange, which is why the block is
// 20 rather than 21 segments.

/// The 20 segments of one hand, in the order MVN streams them.
inline const std::array<const char *, 20> & handSegmentSuffixes()
{
  static const std::array<const char *, 20> kSuffixes = {{
    "carpus",
    "first_mc", "first_pp", "first_dp",
    "second_mc", "second_pp", "second_mp", "second_dp",
    "third_mc", "third_pp", "third_mp", "third_dp",
    "fourth_mc", "fourth_pp", "fourth_mp", "fourth_dp",
    "fifth_mc", "fifth_pp", "fifth_mp", "fifth_dp"}};
  return kSuffixes;
}

/// \param side "left" or "right"
/// \param position 0-based index of the segment within this hand's block,
///        in the order received (ascending segment ID)
/// \return e.g. "left_first_mc", or "" if the position is out of range.
inline std::string fingerSegmentName(const std::string & side, int position)
{
  const auto & suffixes = handSegmentSuffixes();
  if (position < 0 || position >= static_cast<int>(suffixes.size())) {
    return std::string();
  }
  return side + "_" + suffixes[position];
}

/// Parses a name produced by fingerSegmentName() back into its side and its
/// 0-based index within the hand block.  Returns false if `seg` is not a hand
/// segment name.
inline bool parseFingerSegmentName(const std::string & seg, std::string & side, int & position)
{
  for (const char * s : {"left", "right"}) {
    const std::string prefix = std::string(s) + "_";
    if (seg.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string suffix = seg.substr(prefix.size());
    const auto & suffixes = handSegmentSuffixes();
    for (size_t i = 0; i < suffixes.size(); ++i) {
      if (suffix == suffixes[i]) {
        side = s;
        position = static_cast<int>(i);
        return true;
      }
    }
    return false;
  }
  return false;
}

/// Returns the canonical snake_case parent segment name for a given canonical segment name.
/// Returns an empty string for the root segment (pelvis), which is broadcast directly to world.
inline std::string kineticParent(const std::string & seg)
{
  static const std::unordered_map<std::string, std::string> kParent = {
    {"l5", "pelvis"}, {"l3", "l5"}, {"t12", "l3"}, {"t8", "t12"},
    {"neck", "t8"}, {"head", "neck"},
    {"left_shoulder", "t8"}, {"left_upper_arm", "left_shoulder"},
    {"left_forearm", "left_upper_arm"}, {"left_hand", "left_forearm"},
    {"right_shoulder", "t8"}, {"right_upper_arm", "right_shoulder"},
    {"right_forearm", "right_upper_arm"}, {"right_hand", "right_forearm"},
    {"left_upper_leg", "pelvis"}, {"left_lower_leg", "left_upper_leg"},
    {"left_foot", "left_lower_leg"}, {"left_toe", "left_foot"},
    {"right_upper_leg", "pelvis"}, {"right_lower_leg", "right_upper_leg"},
    {"right_foot", "right_lower_leg"}, {"right_toe", "right_foot"},
  };
  const auto it = kParent.find(seg);
  if (it != kParent.end()) {
    return it->second;
  }

  // Dynamic fallback for finger-tracking segments.  The carpus hangs off the
  // hand, as does each finger's metacarpal; every other bone hangs off the
  // previous one in its own finger.
  std::string side;
  int position = 0;
  if (parseFingerSegmentName(seg, side, position)) {
    constexpr int kCarpus = 0;
    constexpr int kThumbMetacarpal = 1;
    if (position == kCarpus) {
      return side + "_hand";
    }
    if (position == kThumbMetacarpal) {
      return side + "_carpus";
    }
    // The index/middle/ring/little metacarpals open each 4-segment group at
    // positions 4, 8, 12 and 16.
    if (position % 4 == 0) {
      return side + "_hand";
    }
    return side + "_" + handSegmentSuffixes()[position - 1];
  }

  return std::string();
}

struct XsensModelNames
{
  const std::vector<std::string> links = {
    "base_link",
    "pelvis",
    "l5",
    "l3",
    "t12",
    "t8",
    "neck",
    "head",
    "right_shoulder",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
    "left_shoulder",
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_leg",
    "right_lower_leg",
    "right_foot",
    "right_toe",
    "left_upper_leg",
    "left_lower_leg",
    "left_foot",
    "left_toe",
    "generic_link"};

  const std::vector<std::string> joints = {
    "l5_s1",
    "l4_l3",
    "l1_t12",
    "t9_t8",
    "t1_c7",
    "c1_head",
    "right_c7_shoulder",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
    "left_c7_shoulder",
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "right_hip",
    "right_knee",
    "right_ankle",
    "right_ballfoot",
    "left_hip",
    "left_knee",
    "left_ankle",
    "left_ballfoot",
    "t8_head_NA",
    "t8_left_upper_arm_NA",
    "t8_right_upper_arm_NA",
    "pelvis_t8_NA",
    "pelvis_pelvis_NA",
    "pelvis_t8_v2_NA"};
};
