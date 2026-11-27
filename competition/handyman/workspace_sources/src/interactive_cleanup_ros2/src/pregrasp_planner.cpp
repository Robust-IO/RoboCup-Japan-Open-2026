#include "interactive_cleanup/pregrasp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace interactive_cleanup
{

namespace
{

struct ModeProfile
{
  const char *mode;
  double camera_clearance;
  double nav_standoff;
  double stop_distance;
  double desired_camera_x;
  double desired_hand_pitch;
  double lift_min;
  double lift_max;
  double flex_min;
  double flex_max;
  const char *approach_profile;
};

const ModeProfile *profileForMode(const std::string &grasp_mode)
{
  static const ModeProfile kProfiles[] = {
    {"floor_pick", 0.10, 0.80, 0.38, 0.48, 1.57, 0.00, 0.18, -2.45, -1.85, "floor_servo"},
    {"table_low_pick", 0.12, 0.72, 0.42, 0.54, 1.57, 0.05, 0.28, -2.10, -1.20, "table_low_servo"},
    {"table_mid_pick", 0.14, 0.66, 0.46, 0.58, 1.40, 0.12, 0.40, -1.80, -0.80, "table_mid_servo"},
  };

  for (const auto &profile : kProfiles) {
    if (grasp_mode == profile.mode) {
      return &profile;
    }
  }
  return nullptr;
}

double wristFlexForDesiredHandPitch(double arm_flex, double desired_hand_pitch)
{
  return -arm_flex - desired_hand_pitch;
}

}  // namespace

PregraspPlan planPregrasp(
  const geometry_msgs::msg::Point &target_in_base,
  const std::string &grasp_mode)
{
  PregraspPlan plan;
  const ModeProfile *profile = profileForMode(grasp_mode);
  if (profile == nullptr) {
    return plan;
  }

  const double desired_height = std::clamp(
    target_in_base.z + profile->camera_clearance,
    0.02,
    0.55);
  const double lateral_bonus = std::min(std::abs(target_in_base.y) * 0.12, 0.06);

  double best_cost = std::numeric_limits<double>::infinity();
  HsrArmPose best_pose;

  for (double arm_lift = profile->lift_min; arm_lift <= profile->lift_max + 1e-6; arm_lift += 0.01) {
    for (double arm_flex = profile->flex_min; arm_flex <= profile->flex_max + 1e-6; arm_flex += 0.02) {
      HsrArmPose candidate;
      candidate.arm_lift = arm_lift;
      candidate.arm_flex = arm_flex;
      candidate.wrist_flex = wristFlexForDesiredHandPitch(
        arm_flex, profile->desired_hand_pitch);

      const auto clamped_candidate = clampArmPoseToLimits(candidate);
      const auto estimate = estimateManipulatorPose(clamped_candidate);
      const double height_error = std::abs(estimate.hand_camera.z - desired_height);
      const double forward_error = std::abs(estimate.hand_camera.x - profile->desired_camera_x);
      const double below_target_penalty =
        estimate.hand_camera.z + 0.01 < target_in_base.z ? 1.0 : 0.0;
      const double pitch_error = std::abs(
        estimateHandPitch(clamped_candidate) - profile->desired_hand_pitch);
      const double cost =
        height_error * 6.0 +
        forward_error * 2.0 +
        below_target_penalty * 5.0 +
        pitch_error * 1.5;

      if (cost >= best_cost) {
        continue;
      }

      best_cost = cost;
      best_pose = clamped_candidate;
    }
  }

  if (!std::isfinite(best_cost)) {
    return plan;
  }

  plan.valid = true;
  plan.grasp_mode = grasp_mode;
  plan.arm_pose = clampArmPoseToLimits(best_pose);
  plan.nav_standoff = profile->nav_standoff + lateral_bonus;
  plan.approach_stop_distance = profile->stop_distance;
  plan.desired_hand_camera_height = desired_height;
  plan.approach_profile = profile->approach_profile;
  return plan;
}

}  // namespace interactive_cleanup
