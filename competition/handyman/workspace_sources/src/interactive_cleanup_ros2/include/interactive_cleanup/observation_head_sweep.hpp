#pragma once

#include <cstddef>
#include <vector>

namespace interactive_cleanup
{

struct ObservationHeadSweepPhase
{
  double pan{0.0};
  double tilt{0.0};
  double motion_duration_sec{0.0};
  double observe_duration_sec{0.0};
};

struct ObservationHeadSweepPlan
{
  std::vector<ObservationHeadSweepPhase> phases;
  double total_duration_sec{0.0};
};

struct ObservationHeadSweepSample
{
  std::size_t phase_index{0};
  double pan{0.0};
  double tilt{0.0};
  bool collecting{false};
  bool complete{false};
};

ObservationHeadSweepPlan buildObservationHeadSweepPlan(
  double observation_tilt,
  double pan_amplitude,
  double motion_duration_sec,
  double observe_duration_sec);

ObservationHeadSweepSample sampleObservationHeadSweep(
  const ObservationHeadSweepPlan &plan,
  double elapsed_sec);

}  // namespace interactive_cleanup
