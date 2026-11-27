#include "interactive_cleanup/observation_head_sweep.hpp"

#include <algorithm>
#include <array>

namespace interactive_cleanup
{

ObservationHeadSweepPlan buildObservationHeadSweepPlan(
  double observation_tilt,
  double pan_amplitude,
  double motion_duration_sec,
  double observe_duration_sec)
{
  ObservationHeadSweepPlan plan;

  const double clamped_motion = std::max(0.0, motion_duration_sec);
  const double clamped_observe = std::max(0.0, observe_duration_sec);
  const double clamped_pan = std::max(0.0, pan_amplitude);
  const std::array<double, 5> pans = {
    0.0,
    clamped_pan,
    0.0,
    -clamped_pan,
    0.0,
  };

  plan.phases.reserve(pans.size());
  for (const double pan : pans) {
    ObservationHeadSweepPhase phase;
    phase.pan = pan;
    phase.tilt = observation_tilt;
    phase.motion_duration_sec = clamped_motion;
    phase.observe_duration_sec = clamped_observe;
    plan.total_duration_sec += clamped_motion + clamped_observe;
    plan.phases.push_back(phase);
  }

  return plan;
}

ObservationHeadSweepSample sampleObservationHeadSweep(
  const ObservationHeadSweepPlan &plan,
  double elapsed_sec)
{
  ObservationHeadSweepSample sample;
  if (plan.phases.empty()) {
    sample.complete = true;
    return sample;
  }

  const double clamped_elapsed = std::max(0.0, elapsed_sec);
  double accumulated = 0.0;

  for (std::size_t index = 0; index < plan.phases.size(); ++index) {
    const auto &phase = plan.phases[index];
    const double phase_duration =
      std::max(0.0, phase.motion_duration_sec) + std::max(0.0, phase.observe_duration_sec);

    if (clamped_elapsed < accumulated + phase_duration) {
      const double phase_elapsed = clamped_elapsed - accumulated;
      sample.phase_index = index;
      sample.pan = phase.pan;
      sample.tilt = phase.tilt;
      sample.collecting = phase_elapsed >= std::max(0.0, phase.motion_duration_sec);
      return sample;
    }

    accumulated += phase_duration;
  }

  const auto &last_phase = plan.phases.back();
  sample.phase_index = plan.phases.size() - 1;
  sample.pan = last_phase.pan;
  sample.tilt = last_phase.tilt;
  sample.collecting = false;
  sample.complete = true;
  return sample;
}

}  // namespace interactive_cleanup
