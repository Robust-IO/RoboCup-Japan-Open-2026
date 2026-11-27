#pragma once
#include <set>
#include <string>
namespace handyman_rebuild_ros2 {
class CancellationBarrier {
public:
  void begin() { waiting_for_handle_=true; }
  void event(const std::string & id,const std::string & state) {
    if (state=="cancel_requested") {
      waiting_for_handle_=false;pending_.insert(id);
    } else if (pending_.count(id)) {
      if (state=="cancel_confirmed") pending_.erase(id);
      else if (state=="cancel_rejected_or_not_acknowledged" ||
        state=="cancel_confirmation_timeout" || state=="goal_ended_otherwise") fault_=true;
    }
  }
  bool blocked() const { return waiting_for_handle_ || fault_ || !pending_.empty(); }
private:
  bool waiting_for_handle_{false},fault_{false};
  std::set<std::string> pending_;
};
}
