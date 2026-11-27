#pragma once
#include "handyman_rebuild_ros2/search_supervisor_lease.hpp"
#include "handyman_rebuild_ros2/search_goal_registration.hpp"
#include "handyman_rebuild_ros2/search_cancel_reporter.hpp"

namespace handyman_rebuild_ros2 {
// Single task / configured point, single-threaded executor. No optional safety
// switches. This is not map authorization; caller must validate the live map.
// Do not dispatch through the external nav reference or replace its observer.
class SearchExecutionGuard {
public:
  struct Target {std::string environment,room,point_id;size_t index;};
  enum class MapState {pending,verified,revoked};
  struct MapEvidence {MapState state;std::string task_id,point_id,digest;};
  using MapCheck=std::function<MapEvidence()>;
  struct Topics {
    std::string lease_requests,lease_replies,registrations,acknowledgements,
      cancellations,statuses;
  };
  SearchExecutionGuard(rclcpp::Node * node,NavigationExecutor & nav,
    const std::string & task,const Target & target,const std::string & digest,
    const Topics & topics,MapCheck map_check)
  : nav_(nav),target_(validate(target)),task_(task),digest_(digest),map_check_(std::move(map_check)),
    lease_(node,nav,task,{topics.lease_requests,topics.lease_replies,topics.statuses}),
    registration_(node,nav,task,target_.point_id,digest,
      {topics.registrations,topics.acknowledgements,topics.statuses},true,true,[this]() {
        return !reporter_.cancelling() && checkMap() && lease_.dispatchAllowed();
      }),
    reporter_(node,nav,task,topics.cancellations,topics.statuses,[this]() {
      // Reporter seals nav before this callback. Expected cancellation ends
      // lease renewal, but never erases an already expired lease fault.
      lease_.retireAfterSeal();
      if (map_failed_ || lease_.failed() || !registration_.healthy()) return std::string("cancel_failed");
      return std::string(registration_.settled() ? "cancel_drained" : "cancel_received");
    }) {
      if (!map_check_) throw std::invalid_argument("map checker required");
      status_=node->create_publisher<handyman_msgs::msg::HandymanMsg>(topics.statuses,10);
      timer_=node->create_wall_timer(std::chrono::milliseconds(100),[this]() {
        if (!reporter_.cancelling()) checkMap();
        if (!map_failed_) return;
        YAML::Node row;row["schema"]="handyman-search-worker-fault-v1";
        row["task_id"]=task_;row["reason"]="map_verification_revoked";
        handyman_msgs::msg::HandymanMsg msg;msg.message="search_worker_fault";msg.detail=YAML::Dump(row);
        status_->publish(msg);
      });
    }
  SearchExecutionGuard(const SearchExecutionGuard &)=delete;
  SearchExecutionGuard & operator=(const SearchExecutionGuard &)=delete;
  bool ready() {return !sent_ && !reporter_.cancelling() && checkMap() && registration_.healthy() && lease_.dispatchAllowed();}
  bool cancelling() const {return reporter_.cancelling();}
  bool retirementReady(const std::string & cancel) {
    return reporter_.drainedFor(cancel) && !map_failed_ && !lease_.failed() &&
      registration_.settled() && nav_.cancellationDrainState()=="cancel_drained";
  }
  template<class Binding,class Completion>
  bool navigate(Binding binding,Completion completion) {
    if (!ready()) return false;
    sent_=true;
    return nav_.navigateToSearchPoint(target_.environment,target_.room,target_.index,binding,completion);
  }
private:
  static Target validate(const Target & target) {
    if (target.environment.empty() || target.room.empty() ||
      target.point_id!=target.environment+"/"+target.room+"/"+std::to_string(target.index))
      throw std::invalid_argument("search target identity mismatch");
    return target;
  }
  bool checkMap() {
    if (map_failed_) return false;
    bool verified=false,invalid=false;
    try {
      const auto evidence=map_check_();
      verified=evidence.state==MapState::verified && evidence.task_id==task_ &&
        evidence.point_id==target_.point_id && evidence.digest==digest_;
      invalid=evidence.state==MapState::revoked ||
        (evidence.state==MapState::verified && !verified) || (sent_ && !verified);
    } catch (...) {invalid=true;}
    if (invalid) {map_failed_=true;nav_.sealAndCancel();}
    return verified && !map_failed_;
  }
  NavigationExecutor & nav_;
  const Target target_;
  const std::string task_,digest_;
  MapCheck map_check_;
  SearchSupervisorLease lease_;
  SearchGoalRegistration registration_;
  SearchCancelReporter reporter_;
  bool sent_=false;
  bool map_failed_=false;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
