#include <cassert>
#include "handyman_rebuild_ros2/search_cancellation_barrier.hpp"
using handyman_rebuild_ros2::SearchCancellationBarrier;
int main() {
  using B = SearchCancellationBarrier;
  const auto t = B::Clock::now();
  B b; assert(!b.blocked(t)); b.begin("task", "cancel", t);
  assert(!b.event("other", "cancel", "cancel_drained", t));
  assert(!b.event("task", "old", "cancel_drained", t));
  assert(b.event("task", "cancel", "cancel_received", t)); assert(b.blocked(t));
  assert(b.event("task", "cancel", "cancel_drained", t)); assert(!b.blocked(t));
  for (bool timeout : {false, true}) {
    B fault; fault.begin("task", "cancel", t);
    auto late = t + std::chrono::seconds(4);
    if (!timeout) fault.event("task", "cancel", "cancel_failed", t);
    fault.event("task", "cancel", "cancel_drained", timeout ? late : t);
    assert(fault.blocked(late));
  }
  B duplicate; duplicate.begin("a", "1", t); duplicate.begin("b", "2", t);
  duplicate.event("a", "1", "cancel_drained", t); assert(duplicate.blocked(t));
  B worker_fault; worker_fault.latchFault(); worker_fault.begin("task", "cancel", t);
  worker_fault.event("task", "cancel", "cancel_drained", t);
  assert(worker_fault.blocked(t));
}
