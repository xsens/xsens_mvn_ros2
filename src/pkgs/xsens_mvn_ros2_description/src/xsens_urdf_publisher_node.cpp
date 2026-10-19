// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>

#include <algorithm>
#include <array>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>                                                // NOLINT(build/include_order)
#include <tf2_ros/transform_listener.h>                                    // NOLINT(build/include_order)
#include <xsens_mvn_ros2_description/segment_defs.hpp>
#include <xsens_mvn_ros2_description/xsens_urdf_publisher_parameters.hpp>

namespace
{

// Reference constants from HumanoidSkeleton (BSN stack), humanoidskeleton.cpp.
//
//   kRefHipWidth = |jRightHip.y| * 2  (jRightHip = {0, -0.08, 0.901})
//               = 0.16 m  — bilateral hip-joint-centre lateral distance in the
//                 reference mesh.
//
// Pelvis Y scaling (matching BSN scalePelvis S[1][1]):
//   sy = actual_bilateralHip_Y / kRefHipWidth
//   actual_bilateralHip_Y is derived from TF:
//     |TF(pelvis→left_upper_leg).y| + |TF(pelvis→right_upper_leg).y|
//   because the left/right upper-leg segment origins ARE the hip joint centres.
//
// Foot scaling — why footSize is NOT used here:
//   kRefFootLength (0.247 m) is the reference heel-to-toe TIP distance, but the
//   foot mesh is anchored at the ANKLE (not the heel) and refLength = 0.1526 m is
//   the ankle-to-ball-of-foot X distance.  Using footSize/kRefFootLength as the
//   scale would make the foot mesh overshoot the ball-of-foot TF joint by ~14 mm,
//   causing a visible gap/overlap with the toe mesh.
//   The XME solver already incorporates bodyDimension.footSize when computing joint
//   positions, so TF(foot→toe).x inherently reflects the subject's foot length.
//   Using actual_TF.x / refLength (the simple-publisher approach) is therefore
//   both correct and body-aware — no additional footSize ratio is needed.
//
// BSN scaleT8() uses S.setIdentity(TF_length_ratio) — plain uniform TF scaling,
// no shoulderWidth involvement.  We match this by falling through to the generic
// Z-only branch (single-axis approximation of BSN's uniform scale).
constexpr double kRefHipWidth = 0.16;   // bilateral hip joint centre distance (m)

using xsens_mvn_ros2_description::kFingerSegments;
using xsens_mvn_ros2_description::kSegments;
using xsens_mvn_ros2_description::ScaleAxis;
using xsens_mvn_ros2_description::SegmentDef;

/// True for the MANUS finger segments, i.e. any name in the finger table.
/// Deriving this from the table itself keeps the one source of truth local to
/// this package and avoids a second copy of MVN's hand-segment naming.
bool isFingerSegment(const char * name)
{
  const std::string n(name);
  return std::any_of(
    kFingerSegments.begin(), kFingerSegments.end(),
    [&](const SegmentDef & seg) {return n == seg.name;});
}

/// Per-segment anisotropic scale (sx, sy, sz).
using Scale3 = std::array<double, 3>;

/// Build the URDF scale attribute "sx sy sz".
std::string scaleString(double sx, double sy, double sz)
{
  std::ostringstream ss;
  ss << sx << " " << sy << " " << sz;
  return ss.str();
}

/// Scale a "x y z" visual-origin string by independent per-axis factors.
std::string scaleVisualXyz(const char * xyzStr, double sx, double sy, double sz)
{
  double x = 0.0, y = 0.0, z = 0.0;
  std::istringstream iss(xyzStr);
  iss >> x >> y >> z;

  std::ostringstream oss;
  oss << (x * sx) << " " << (y * sy) << " " << (z * sz);
  return oss.str();
}

}  // namespace

class XsensUrdfPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  XsensUrdfPublisher()
  : LifecycleNode("xsens_urdf_publisher")
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    m_paramListener = std::make_shared<xsens_urdf_publisher::ParamListener>(
      get_node_parameters_interface());
    const auto params = m_paramListener->get_params();
    m_modelName = params.model_name;
    m_republishPeriodS = params.republish_period;

    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // Transient-local (latched) so that RViz and other late subscribers receive
    // the message after it was first published.
    m_robotDescriptionPub = create_publisher<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local());

    // Service for on-demand re-publication (e.g. after a new user puts on the suit).
    m_republishSrv = create_service<std_srvs::srv::Trigger>(
      "~/republish_urdf",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          response->success = false;
          response->message = "Node is not active.";
          return;
        }
        RCLCPP_INFO(get_logger(), "URDF re-publish requested via service.");
        if (tryPublish()) {
          response->success = true;
          response->message = "URDF re-published.";
        } else {
          response->success = false;
          response->message = "TF data not yet available; URDF not published.";
        }
      });

    // Diagnostics
    m_diagnosticUpdater = std::make_unique<diagnostic_updater::Updater>(this);
    m_diagnosticUpdater->setHardwareID("Xsens URDF Publisher");
    m_diagnosticUpdater->add(
      "URDF publisher status", this,
      &XsensUrdfPublisher::diagnosticsCallback);
    m_diagnosticTimer = create_wall_timer(
      std::chrono::seconds(1), [this]() {m_diagnosticUpdater->force_update();});

    RCLCPP_INFO(get_logger(), "Configured. Waiting for activation...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    m_robotDescriptionPub->on_activate();

    // Fast poll until first successful publish, then slow down to periodic updates.
    m_timer = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {tryPublish();});

    RCLCPP_INFO(get_logger(), "Activated. Waiting for TF data to compute segment scales...");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (m_timer) {
      m_timer->cancel();
      m_timer.reset();
    }
    m_robotDescriptionPub->on_deactivate();
    m_published = false;

    RCLCPP_INFO(get_logger(), "Deactivated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    m_timer.reset();
    m_diagnosticTimer.reset();
    m_diagnosticUpdater.reset();
    m_robotDescriptionPub.reset();
    m_republishSrv.reset();
    m_tfListener.reset();
    m_tfBuffer.reset();
    m_paramListener.reset();
    m_published = false;
    m_clampedSegments = 0;

    RCLCPP_INFO(get_logger(), "Cleaned up.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    m_timer.reset();
    m_diagnosticTimer.reset();
    m_diagnosticUpdater.reset();
    m_robotDescriptionPub.reset();
    m_republishSrv.reset();
    m_tfListener.reset();
    m_tfBuffer.reset();
    m_paramListener.reset();

    RCLCPP_INFO(get_logger(), "Shut down.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override
  {
    m_timer.reset();
    m_diagnosticTimer.reset();
    m_diagnosticUpdater.reset();
    m_robotDescriptionPub.reset();
    m_republishSrv.reset();
    m_tfListener.reset();
    m_tfBuffer.reset();
    m_paramListener.reset();
    m_published = false;

    RCLCPP_ERROR(get_logger(), "Error encountered, returning to unconfigured.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<xsens_urdf_publisher::ParamListener> m_paramListener;
  std::string m_modelName;
  double m_republishPeriodS{10.0};
  bool m_published = false;
  std::set<std::string> m_fingersActive;  //!< Sides with a live MANUS finger block
  int m_clampedSegments = 0;
  rclcpp::Time m_lastPublishTime{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tfListener;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr m_robotDescriptionPub;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::TimerBase::SharedPtr m_diagnosticTimer;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_republishSrv;
  std::unique_ptr<diagnostic_updater::Updater> m_diagnosticUpdater;

  /// True once the stream node has started broadcasting finger frames for
  /// this hand.  The carpus is the first segment of the hand's block,
  /// so its presence means the whole block is being published.
  bool hasFingers(const std::string & side)
  {
    return m_tfBuffer->canTransform(
      m_modelName + "_" + side + "_hand",
      m_modelName + "_" + side + "_carpus",
      tf2::TimePointZero);
  }

  /// The closed-fist hand mesh spans the whole hand including fingers, so it
  /// would intersect the per-phalange meshes.  Drop it on a side whose finger
  /// block is live; the carpus mesh draws the palm instead.
  bool suppressVisual(const char * segName) const
  {
    const std::string n(segName);
    return (n == "left_hand" && m_fingersActive.count("left") > 0) ||
           (n == "right_hand" && m_fingersActive.count("right") > 0);
  }

  /// The body segments, plus each hand's finger block when MANUS data is live.
  /// Recomputed on every publish so gloves connecting (or dropping out) mid
  /// session are picked up on the next re-publish.
  std::vector<SegmentDef> buildActiveSegments()
  {
    std::vector<SegmentDef> segs(kSegments.begin(), kSegments.end());
    m_fingersActive.clear();
    for (const auto & side : {std::string("left"), std::string("right")}) {
      if (!hasFingers(side)) {
        continue;
      }
      for (const auto & f : kFingerSegments) {
        if (std::string(f.name).rfind(side + "_", 0) == 0 && isFingerSegment(f.name)) {
          segs.push_back(f);
        }
      }
      m_fingersActive.insert(side);
    }
    return segs;
  }

  /// Returns the distance (metres) between two segment TF frame origins.
  /// When axis is X, Y, or Z the absolute value of that component is returned;
  /// otherwise (Uniform) the Euclidean distance is used.
  /// Returns -1.0 if the transform is not yet available.
  double segmentLength(
    const std::string & fromSeg, const std::string & toSeg, ScaleAxis axis)
  {
    try {
      auto tf = m_tfBuffer->lookupTransform(
        m_modelName + "_" + fromSeg,
        m_modelName + "_" + toSeg,
        tf2::TimePointZero);
      const auto & t = tf.transform.translation;
      switch (axis) {
        case ScaleAxis::X: return std::abs(t.x);
        case ScaleAxis::Y: return std::abs(t.y);
        case ScaleAxis::Z: return std::abs(t.z);
        default:           return std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        get_logger(), "TF lookup failed for '%s_%s' -> '%s_%s': %s",
        m_modelName.c_str(), fromSeg.c_str(),
        m_modelName.c_str(), toSeg.c_str(), ex.what());
      return -1.0;
    }
  }

  /// Tries to compute all Scale3 values from TF and body dimension parameters;
  /// on success publishes the URDF.  After the first successful publish, switches
  /// the timer to a slow periodic re-publish so body dimension changes are picked up.
  /// Returns true if the URDF was published successfully.
  ///
  /// Per-segment scaling algorithm (based on BSN HumanMeshInfo, humanmeshinfo.cpp):
  ///
  ///   pelvis  : sy = bilateral hip-joint-centre TF distance / kRefHipWidth (0.16 m)
  ///             (matches BSN scalePelvis S[1][1] = bilateral_actual / 0.16)
  ///             sx = 1.0  (BSN uses ASIS-SIPS ratio; unavailable here)
  ///             sz = TF(pelvis→l5) / refLength
  ///
  ///   *_foot  : s = TF.x / refLength  (uniform {s,s,s}, same as simple publisher)
  ///             footSize is already encoded in TF by the XME solver; applying
  ///             an additional footSize/kRefFootLength ratio would mix reference
  ///             frames and overshoot the ball-of-foot joint by ~14 mm.
  ///
  ///   scaleAxis=Y (arms, shoulders):
  ///             sy = TF / ref,  sx = sz = 1.0
  ///
  ///   all other (spine, t8, legs — BSN uses uniform TF scale for these):
  ///             sz = TF / ref,  sx = sy = 1.0
  ///
  ///   leaf segments: inherit parent Scale3 unchanged.
  ///
  /// Each component is clamped independently to [0.5, 2.0].
  bool tryPublish()
  {
    const std::vector<SegmentDef> segs = buildActiveSegments();
    std::vector<Scale3> scales(segs.size(), {1.0, 1.0, 1.0});

    // Pass 1: compute anisotropic Scale3 for every non-leaf segment.
    int clampedCount = 0;
    auto clampAndTrack = [&](double value, const char * segName, const char * axisLabel) {
        double clamped = std::clamp(value, 0.5, 2.0);
        if (clamped != value) {
          clampedCount++;
          RCLCPP_WARN_ONCE(
          get_logger(), "Scale clamped for '%s' (%s): %.3f -> %.3f. "
          "This may indicate unusual body proportions or TF data issues.",
          segName, axisLabel, value, clamped);
        }
        return clamped;
      };

    for (size_t i = 0; i < segs.size(); ++i) {
      const auto & seg = segs[i];
      if (std::string(seg.child).empty() || seg.refLength <= 0.0) {
        continue;  // leaf, or no mesh to scale — handled in pass 2
      }

      double actual = segmentLength(seg.name, seg.child, seg.distanceAxis);
      if (actual < 0.0) {
        // A missing finger transform must not hold back the whole body URDF:
        // fall back to the neutral mesh size and keep going.  Body segments
        // stay strict, since the model is meaningless without them.
        if (isFingerSegment(seg.name)) {
          continue;
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for TF: %s_%s -> %s_%s",
          m_modelName.c_str(), seg.name,
          m_modelName.c_str(), seg.child);
        return false;
      }

      const std::string name(seg.name);

      if (name == "pelvis") {
        // Y scale: bilateral hip-joint-centre lateral distance from TF.
        // left_upper_leg and right_upper_leg origins ARE the hip joint centres,
        // so this directly matches BSN scalePelvis S[1][1].
        double leftHipY = segmentLength("pelvis", "left_upper_leg", ScaleAxis::Y);
        double rightHipY = segmentLength("pelvis", "right_upper_leg", ScaleAxis::Y);
        if (leftHipY < 0.0 || rightHipY < 0.0) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for TF: %s_pelvis -> left/right_upper_leg",
            m_modelName.c_str());
          return false;
        }
        double bilateralHip = leftHipY + rightHipY;
        double sy = clampAndTrack(bilateralHip / kRefHipWidth, seg.name, "Y");
        double sz = clampAndTrack(actual / seg.refLength, seg.name, "Z");
        scales[i] = {1.0, sy, sz};

      } else if (name == "left_foot" || name == "right_foot") {
        // Scale uniformly from TF.x (ankle→ball-of-foot), matching the simple
        // publisher.  The XME solver already incorporates bodyDimension.footSize
        // when computing TF joint positions, so this is inherently body-aware.
        // Applying footSize/kRefFootLength separately would mismatch reference
        // frames (heel-to-toe vs ankle-to-ball) and overshoot the ball-of-foot
        // joint, breaking the visual connection with the toe mesh.
        double s = clampAndTrack(actual / seg.refLength, seg.name, "uniform");
        scales[i] = {s, s, s};

      } else if (seg.scaleAxis == ScaleAxis::Y) {
        // Arms and shoulders: scale along Y (primary elongation axis).
        double sy = clampAndTrack(actual / seg.refLength, seg.name, "Y");
        scales[i] = {1.0, sy, 1.0};

      } else {
        // Spine (l5, l3, t12, t8, neck), legs: scale along Z.
        // BSN uses S.setIdentity (uniform TF scale) for these; single-axis Z
        // is a close approximation that avoids over-widening when TF length > ref.
        double sz = clampAndTrack(actual / seg.refLength, seg.name, "Z");
        scales[i] = {1.0, 1.0, sz};
      }
    }
    m_clampedSegments = clampedCount;

    // Pass 2: leaf segments inherit the full Scale3 of their parent.
    for (size_t i = 0; i < segs.size(); ++i) {
      if (!std::string(segs[i].child).empty()) {
        continue;
      }
      for (size_t p = 0; p < segs.size(); ++p) {
        if (std::string(segs[p].name) == std::string(segs[i].parent)) {
          scales[i] = scales[p];
          break;
        }
      }
    }

    publishUrdf(segs, scales);

    if (!m_published) {
      m_published = true;
      m_timer->cancel();
      m_timer = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(m_republishPeriodS)),
        [this]() {tryPublish();});
      RCLCPP_INFO(get_logger(), "URDF will re-publish every %.1f seconds.", m_republishPeriodS);
    }
    return true;
  }

  void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (!m_published) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "URDF not yet published — waiting for TF data.");
      return;
    }

    auto elapsed = (now() - m_lastPublishTime).seconds();
    stat.add("Last published (s ago)", static_cast<int>(elapsed));
    stat.add("Clamped segments", m_clampedSegments);
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "URDF published for model '" + m_modelName + "'.");

    if (m_clampedSegments > 0) {
      stat.mergeSummary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::to_string(m_clampedSegments) +
          " segment(s) had scale values clamped to [0.5, 2.0].");
    }

    // Warn if TF data is significantly stale (more than 3× the republish period).
    if (elapsed > 3.0 * m_republishPeriodS) {
      stat.mergeSummary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "URDF not re-published in " + std::to_string(static_cast<int>(elapsed)) + " s.");
    }
  }

  void publishUrdf(const std::vector<SegmentDef> & segs, const std::vector<Scale3> & scales)
  {
    std::ostringstream xml;
    // The robot name must be unique per model.  RViz keys its RobotModel
    // display off it, so two avatars both called "humanoid" collide: one
    // display renders and the other reports an invalid model until the first
    // is switched off.  Naming it after the model keeps every avatar distinct.
    xml << "<?xml version=\"1.0\"?>\n"
        << "<robot name=\"" << m_modelName << "\">\n\n"
        << "  <!-- World link (reference frame). -->\n"
        << "  <link name=\"world\"/>\n\n"
        << "  <!-- Segments form a kinematic tree rooted at pelvis.\n"
        << "       TF broadcasts from the Xsens node supply relative transforms.\n"
        << "       Mesh scaling (MVN legacy mode):\n"
        << "         - Pelvis Y: bilateral hip-joint-centre TF distance / 0.16 m ref.\n"
        << "         - Feet X: footSize / 0.247 m ref (or TF.x fallback).\n"
        << "         - All other segments: single-axis TF length scaling. -->\n\n";

    for (size_t i = 0; i < segs.size(); ++i) {
      const auto & seg = segs[i];
      const auto & sc = scales[i];
      const std::string scale = scaleString(sc[0], sc[1], sc[2]);
      const std::string link = m_modelName + "_" + seg.name;
      const std::string parentLink =
        (std::string(seg.parent) == "world") ? "world" : m_modelName + "_" + seg.parent;
      const std::string jointName = m_modelName + "_" + seg.parent + "_to_" + seg.name;

      xml << "  <link name=\"" << link << "\">\n";

      // A link is emitted without geometry when no mesh is shipped for it
      // (the two missing metacarpals), and when the closed-fist hand mesh
      // would sit on top of the individual finger meshes.
      if (std::string(seg.mesh).empty() || suppressVisual(seg.name)) {
        xml << "  </link>\n"
            << "  <joint name=\"" << jointName << "\" type=\"fixed\">\n"
            << "    <parent link=\"" << parentLink << "\"/>\n"
            << "    <child link=\"" << link << "\"/>\n"
            << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            << "  </joint>\n\n";
        continue;
      }

      xml << "    <visual>\n"
          << "      <origin xyz=\"" << scaleVisualXyz(seg.visualXyz, sc[0], sc[1], sc[2])
          << "\" rpy=\"" << seg.visualRpy << "\"/>\n"
          << "      <geometry>\n"
          << "        <mesh filename=\"package://xsens_mvn_ros2_description/meshes/neutral/xsens_"
          << seg.mesh << ".dae\""
          << " scale=\"" << scale << "\"/>\n"
          << "      </geometry>\n"
          << "    </visual>\n";

      // Attach the Torso chest-plate as a second visual on the T8 link,
      // using the same Scale3 as the primary T8 mesh.
      if (std::string(seg.name) == "t8") {
        xml << "    <visual>\n"
            << "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            << "      <geometry>\n"
            << "        <mesh filename=\"package://xsens_mvn_ros2_description"
            << "/meshes/neutral/xsens_Torso.dae\""
            << " scale=\"" << scale << "\"/>\n"
            << "      </geometry>\n"
            << "    </visual>\n";
      }

      xml << "  </link>\n"
          << "  <joint name=\"" << jointName << "\" type=\"fixed\">\n"
          << "    <parent link=\"" << parentLink << "\"/>\n"
          << "    <child link=\"" << link << "\"/>\n"
          << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
          << "  </joint>\n\n";
    }

    xml << "</robot>\n";

    std_msgs::msg::String msg;
    msg.data = xml.str();
    m_robotDescriptionPub->publish(msg);

    m_lastPublishTime = now();
    RCLCPP_INFO(get_logger(),
      "Published robot_description for model '%s'.", m_modelName.c_str());
    for (size_t i = 0; i < segs.size(); ++i) {
      if (!std::string(segs[i].child).empty()) {
        RCLCPP_DEBUG(get_logger(),
          "  %-24s scale = (%.3f, %.3f, %.3f)",
          segs[i].name, scales[i][0], scales[i][1], scales[i][2]);
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XsensUrdfPublisher>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
