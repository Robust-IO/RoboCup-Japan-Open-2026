#pragma once
#include <chrono>
#include <string>

namespace handyman_rebuild_ros2 {
// One external worker owns a request. Drained is a worker-level guarantee:
// all accepted/late goals are terminal and no further goals may be submitted.
class SearchCancellationBarrier {
public:
  using Clock = std::chrono::steady_clock;
  void latchFault() { fault_ = true; }
  void begin(const std::string & task, const std::string & cancellation,
    Clock::time_point now = Clock::now()) {
    if (pending_) { fault_ = true; return; }
    task_ = task; cancellation_ = cancellation; pending_ = true;
    deadline_ = now + std::chrono::seconds(3);
  }
  bool event(const std::string & task, const std::string & cancellation,
    const std::string & state, Clock::time_point now = Clock::now()) {
    check(now);
    if (!pending_ || task != task_ || cancellation != cancellation_) return false;
    if (state == "cancel_drained" && !fault_) pending_ = false;
    else if (state == "cancel_failed") fault_ = true;
    else if (state != "cancel_received") return false;
    return true;
  }
  bool blocked(Clock::time_point now = Clock::now()) {
    check(now); return pending_ || fault_;
  }
private:
  void check(Clock::time_point now) { if (pending_ && now >= deadline_) fault_ = true; }
  std::string task_, cancellation_;
  bool pending_{false}, fault_{false};
  Clock::time_point deadline_{};
};
}
