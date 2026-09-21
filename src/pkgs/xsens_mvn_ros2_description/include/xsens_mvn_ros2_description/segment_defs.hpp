// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>

namespace xsens_mvn_ros2_description
{

/// Which local mesh axis points from the segment origin toward its child joint.
enum class ScaleAxis { X = 0, Y = 1, Z = 2, Uniform = 3 };

struct SegmentDef
{
  const char * name;        //!< Canonical snake_case segment name
  const char * mesh;        //!< PascalCase .dae filename prefix ("" = no visual)
  const char * child;       //!< Canonical child segment ("" = leaf)
  const char * parent;      //!< Kinematic parent name
  double refLength;        //!< Mesh extent along distanceAxis at scale 1.0 (m)
  ScaleAxis scaleAxis;     //!< Primary elongation axis of the mesh
  ScaleAxis distanceAxis;  //!< TF component used to compute the scale factor
  const char * visualRpy;  //!< Visual origin rotation (radians)
  const char * visualXyz;  //!< Visual origin translation (metres)
};

// Body segment table — the 23 standard MVN segments, always present.
static const std::array<SegmentDef, 23> kSegments = {{  // NOLINT
  // Spine
  {"pelvis", "Pelvis", "l5", "world", 0.096, ScaleAxis::Z, ScaleAxis::Uniform, "0 0 0", "0 0 0"},            // NOLINT
  {"l5", "L5", "l3", "pelvis", 0.106, ScaleAxis::Z, ScaleAxis::Uniform, "0 0 0", "0 0 0"},                   // NOLINT
  {"l3", "L3", "t12", "l5", 0.095, ScaleAxis::Z, ScaleAxis::Uniform, "0 0 0", "0 0 0"},                      // NOLINT
  {"t12", "T12", "t8", "l3", 0.094, ScaleAxis::Z, ScaleAxis::Uniform, "0 0 0", "0 0 0"},                     // NOLINT
  {"t8", "T8", "neck", "t12", 0.128, ScaleAxis::Z, ScaleAxis::Uniform, "0 0 0", "0 0 0"},                    // NOLINT
  {"neck", "Neck", "head", "t8", 0.104, ScaleAxis::Z, ScaleAxis::Uniform, "0 0 0", "0 0 0"},                 // NOLINT
  {"head", "Head", "", "neck", 0.0, ScaleAxis::Uniform, ScaleAxis::Uniform, "0 0 0", "0 0 0"},                   // NOLINT
  // Left arm
  {"left_shoulder", "LeftShoulder", "left_upper_arm", "t8", 0.140, ScaleAxis::Y, ScaleAxis::Uniform,
    "0 0 0", "0 0 0"},                                                                                                                // NOLINT
  {"left_upper_arm", "LeftUpperArm", "left_forearm", "left_shoulder", 0.306, ScaleAxis::Y,
    ScaleAxis::Uniform, "-1.5708 0 0", "0 0 0"},                                                                                            // NOLINT
  {"left_forearm", "LeftForeArm", "left_hand", "left_upper_arm", 0.254, ScaleAxis::Y,
    ScaleAxis::Uniform, "-1.5708 0 0", "0 0 0"},                                                                                            // NOLINT
  {"left_hand", "LeftHand", "", "left_forearm", 0.0, ScaleAxis::Uniform, ScaleAxis::Uniform,
    "-1.5708 0 0", "0 0 0"},                                                                                                                    // NOLINT
  // Right arm
  {"right_shoulder", "RightShoulder", "right_upper_arm", "t8", 0.140, ScaleAxis::Y,
    ScaleAxis::Uniform, "0 0 0", "0 0 0"},                                                                                                // NOLINT
  {"right_upper_arm", "RightUpperArm", "right_forearm", "right_shoulder", 0.306, ScaleAxis::Y,
    ScaleAxis::Uniform, "1.5708 0 0", "0 0 0"},                                                                                                // NOLINT
  {"right_forearm", "RightForeArm", "right_hand", "right_upper_arm", 0.254, ScaleAxis::Y,
    ScaleAxis::Uniform, "1.5708 0 0", "0 0 0"},                                                                                                // NOLINT
  {"right_hand", "RightHand", "", "right_forearm", 0.0, ScaleAxis::Uniform, ScaleAxis::Uniform,
    "1.5708 0 0", "0 0 0"},                                                                                                                        // NOLINT
  // Left leg
  {"left_upper_leg", "LeftUpperLeg", "left_lower_leg", "pelvis", 0.417, ScaleAxis::Z,
    ScaleAxis::Uniform, "0 0 0", "0 0 0"},                                                                                            // NOLINT
  {"left_lower_leg", "LeftLowerLeg", "left_foot", "left_upper_leg", 0.408, ScaleAxis::Z,
    ScaleAxis::Uniform, "0 0 0", "0 0 0"},                                                                                            // NOLINT
  {"left_foot", "LeftFoot", "left_toe", "left_lower_leg", 0.1526, ScaleAxis::Uniform, ScaleAxis::X,
    "0 0 0", "0 0 0"},                                                                                                                 // NOLINT
  {"left_toe", "LeftToe", "", "left_foot", 0.0, ScaleAxis::Uniform, ScaleAxis::X, "0 0 0",
    "-0.010 0 -0.015"},                                                                                                                       // NOLINT
  // Right leg
  {"right_upper_leg", "RightUpperLeg", "right_lower_leg", "pelvis", 0.417, ScaleAxis::Z,
    ScaleAxis::Uniform, "0 0 0", "0 0 0"},                                                                                               // NOLINT
  {"right_lower_leg", "RightLowerLeg", "right_foot", "right_upper_leg", 0.408, ScaleAxis::Z,
    ScaleAxis::Uniform, "0 0 0", "0 0 0"},                                                                                                // NOLINT
  {"right_foot", "RightFoot", "right_toe", "right_lower_leg", 0.1526, ScaleAxis::Uniform,
    ScaleAxis::X, "0 0 0", "0 0 0"},                                                                                                       // NOLINT
  {"right_toe", "RightToe", "", "right_foot", 0.0, ScaleAxis::Uniform, ScaleAxis::X, "0 0 0",
    "-0.010 0 -0.015"},                                                                                                                           // NOLINT
}};

// Finger segment table - MANUS glove finger tracking, 20 segments per hand.
//
// Names mirror MVN's own (LeftCarpus, LeftFirstMC, ...) in canonical
// snake_case; see handSegmentSuffixes() in xsens_model.hpp.  The block is not
// a uniform 5 x 4 grid of phalanges but the MVN hand model, which also totals
// 20: carpus + thumb MC/PP/DP + four fingers x MC/PP/MP/DP.  Verified against
// the names MVN reports in its scaling datagram and against live TF bone
// lengths (every scale factor lands in 0.85-0.96).
//
// MVN sends a metacarpal for every finger, but the neutral mesh set ships no
// SecondMC or FifthMC.  Those two links per hand are therefore emitted without
// a visual: the kinematic chain stays intact (their children are placed from
// TF as usual) and the palm geometry is already covered by the carpus plus the
// third and fourth metacarpal meshes.
//
// Mesh convention: every finger mesh is authored in a shared hand frame whose
// origin sits at y = -0.0706 m (left) / +0.0706 m (right).  Translating a mesh
// by that offset puts its proximal end exactly on the joint, which is what
// visualXyz below does; scaleVisualXyz() scales the offset alongside the mesh.
// Bones run +Y on the left hand and -Y on the right, matching the TF data, so
// no visual rotation is needed.
static const std::array<SegmentDef, 40> kFingerSegments = {{  // NOLINT
  // ---- left hand ----
  {"left_carpus", "LeftCarpus", "left_first_mc", "left_hand", 0.0428,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_first_mc", "LeftFirstMC", "left_first_pp", "left_carpus", 0.0385,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_first_pp", "LeftFirstPP", "left_first_dp", "left_first_mc", 0.0345,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_first_dp", "LeftFirstDP", "", "left_first_pp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_second_mc", "", "left_second_pp", "left_hand", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_second_pp", "LeftSecondPP", "left_second_mp", "left_second_mc", 0.0458,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_second_mp", "LeftSecondMP", "left_second_dp", "left_second_pp", 0.0267,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_second_dp", "LeftSecondDP", "", "left_second_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_third_mc", "LeftThirdMC", "left_third_pp", "left_hand", 0.0659,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_third_pp", "LeftThirdPP", "left_third_mp", "left_third_mc", 0.0497,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_third_mp", "LeftThirdMP", "left_third_dp", "left_third_pp", 0.0307,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_third_dp", "LeftThirdDP", "", "left_third_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fourth_mc", "LeftFourthMC", "left_fourth_pp", "left_hand", 0.0663,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fourth_pp", "LeftFourthPP", "left_fourth_mp", "left_fourth_mc", 0.0474,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fourth_mp", "LeftFourthMP", "left_fourth_dp", "left_fourth_pp", 0.0298,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fourth_dp", "LeftFourthDP", "", "left_fourth_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fifth_mc", "", "left_fifth_pp", "left_hand", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fifth_pp", "LeftFifthPP", "left_fifth_mp", "left_fifth_mc", 0.0403,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fifth_mp", "LeftFifthMP", "left_fifth_dp", "left_fifth_pp", 0.0238,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  {"left_fifth_dp", "LeftFifthDP", "", "left_fifth_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 0.0706 0"},
  // ---- right hand ----
  {"right_carpus", "RightCarpus", "right_first_mc", "right_hand", 0.0428,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_first_mc", "RightFirstMC", "right_first_pp", "right_carpus", 0.0385,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_first_pp", "RightFirstPP", "right_first_dp", "right_first_mc", 0.0345,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_first_dp", "RightFirstDP", "", "right_first_pp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_second_mc", "", "right_second_pp", "right_hand", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_second_pp", "RightSecondPP", "right_second_mp", "right_second_mc", 0.0458,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_second_mp", "RightSecondMP", "right_second_dp", "right_second_pp", 0.0267,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_second_dp", "RightSecondDP", "", "right_second_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_third_mc", "RightThirdMC", "right_third_pp", "right_hand", 0.0659,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_third_pp", "RightThirdPP", "right_third_mp", "right_third_mc", 0.0497,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_third_mp", "RightThirdMP", "right_third_dp", "right_third_pp", 0.0307,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_third_dp", "RightThirdDP", "", "right_third_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fourth_mc", "RightFourthMC", "right_fourth_pp", "right_hand", 0.0663,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fourth_pp", "RightFourthPP", "right_fourth_mp", "right_fourth_mc", 0.0474,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fourth_mp", "RightFourthMP", "right_fourth_dp", "right_fourth_pp", 0.0298,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fourth_dp", "RightFourthDP", "", "right_fourth_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fifth_mc", "", "right_fifth_pp", "right_hand", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fifth_pp", "RightFifthPP", "right_fifth_mp", "right_fifth_mc", 0.0403,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fifth_mp", "RightFifthMP", "right_fifth_dp", "right_fifth_pp", 0.0238,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
  {"right_fifth_dp", "RightFifthDP", "", "right_fifth_mp", 0.0000,
    ScaleAxis::Y, ScaleAxis::Uniform, "0 0 0", "0 -0.0706 0"},
}};

}  // namespace xsens_mvn_ros2_description
