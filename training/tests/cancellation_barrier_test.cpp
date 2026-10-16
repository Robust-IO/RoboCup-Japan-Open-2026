#include <cassert>
#include "handyman_rebuild_ros2/cancellation_barrier.hpp"
using handyman_rebuild_ros2::CancellationBarrier;
int main() {
  CancellationBarrier b; assert(!b.blocked());b.begin();assert(b.blocked());
  b.event("wrong","cancel_confirmed");assert(b.blocked());
  b.event("first","cancel_requested");b.event("first","cancel_accepted_waiting_terminal");assert(b.blocked());
  b.event("first","cancel_confirmed");assert(!b.blocked());
  for (auto state:{"cancel_rejected_or_not_acknowledged","cancel_confirmation_timeout","goal_ended_otherwise"}) {
    CancellationBarrier f;f.begin();f.event("one","cancel_requested");f.event("one",state);
    assert(f.blocked());f.event("one","cancel_confirmed");assert(f.blocked());
  }
  CancellationBarrier many;many.begin();many.event("a","cancel_requested");many.event("b","cancel_requested");
  many.event("a","cancel_confirmed");assert(many.blocked());many.event("b","cancel_confirmed");assert(!many.blocked());
}
