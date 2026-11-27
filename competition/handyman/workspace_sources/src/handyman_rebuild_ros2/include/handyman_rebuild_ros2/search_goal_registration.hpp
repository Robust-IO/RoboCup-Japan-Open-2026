#pragma once
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <handyman_msgs/msg/handyman_msg.hpp>
#include "handyman_rebuild_ros2/navigation_executor.hpp"

namespace handyman_rebuild_ros2 {
// One task/search point per executor lifetime; single-threaded callbacks only.
// This object owns the OwnedGoalObserver slot. Keep it alive until nav drains.
// Intent ACK authorizes the original UUID only after the supervisor persisted it.
// Acceptance/rejection remains separately acknowledged for retirement accounting.
class SearchGoalRegistration {
public:
  struct Topics {std::string registrations, acknowledgements, statuses;};
  SearchGoalRegistration(rclcpp::Node * node, NavigationExecutor & nav,
    std::string task, std::string point, std::string digest, const Topics & topics,
    bool require_ack=true,bool require_intent=false,std::function<bool()> dispatch_permit={})
  : node_(node),nav_(nav),task_(std::move(task)),point_(std::move(point)),
    digest_(std::move(digest)),require_ack_(require_ack),dispatch_permit_(std::move(dispatch_permit)) {
    auto hex=[](const std::string & s,size_t n) {
      return s.size()==n && std::all_of(s.begin(),s.end(),[](char c) {
        return (c>='0' && c<='9') || (c>='a' && c<='f');});};
    if (!node || !hex(task_,32) || task_==std::string(32,'0') || point_.empty() ||
      !hex(digest_,64) || topics.registrations.empty() || topics.acknowledgements.empty() ||
      topics.statuses.empty()) throw std::invalid_argument("invalid goal registration owner/topics");
    publisher_=node_->create_publisher<Msg>(topics.registrations,100);
    fault_=node_->create_publisher<Msg>(topics.statuses,10);
    ack_=node_->create_subscription<Msg>(topics.acknowledgements,100,[this](const Msg & msg) {acknowledge(msg);});
    timer_=node_->create_wall_timer(std::chrono::milliseconds(100),[this]() {tick();});
    nav_.setOwnedGoalObserver([this](const auto & uuid,auto generation,const auto & pose,bool waypoint) {
      record("owned_goal_accepted",uuid,generation,pose,waypoint,{});
    });
    if (require_intent) {
      if (!require_ack_) throw std::invalid_argument("dispatch intent requires acknowledgement");
      nav_.setDispatchIntentObserver([this](const auto & uuid,auto gen,const auto & pose,bool waypoint,auto decide) {
        record("owned_goal_intent",uuid,gen,pose,waypoint,decide);
      });
      nav_.setRejectedGoalObserver([this](const auto & uuid,auto gen,const auto & pose,bool waypoint) {
        record("owned_goal_rejected",uuid,gen,pose,waypoint,{});
      });
    }
  }
  ~SearchGoalRegistration() {nav_.setOwnedGoalObserver({});nav_.setDispatchIntentObserver({});nav_.setRejectedGoalObserver({});}
  SearchGoalRegistration(const SearchGoalRegistration &)=delete;
  SearchGoalRegistration & operator=(const SearchGoalRegistration &)=delete;
  bool healthy() {expire();return !failed_;}
  bool settled() {expire();return !failed_ && pending_.empty();}
private:
  using Clock=std::chrono::steady_clock;
  using Msg=handyman_msgs::msg::HandymanMsg;
  struct Pending {Msg message;Clock::time_point deadline;DeferredNavigationClient::Decision decide;};
  void record(const std::string & event,const rclcpp_action::GoalUUID & uuid,
    std::uint64_t generation,const Pose2D & pose,bool waypoint,DeferredNavigationClient::Decision decide) {
    std::ostringstream key;
    for (auto byte:uuid) key<<std::hex<<std::setw(2)<<std::setfill('0')<<int(byte);
    YAML::Node row;
    row["schema"]="handyman-owned-goal-v1";row["task_id"]=task_;
    row["point_id"]=point_;row["map_sha256"]=digest_;row["goal_id"]=key.str();
    row["generation"]=generation;row["stamp_ns"]=node_->now().nanoseconds();
    row["role"]=waypoint ? "route_waypoint" : "final_search_point";row["frame_id"]="map";
    row["pose"]["x"]=pose.x;row["pose"]["y"]=pose.y;row["pose"]["yaw"]=pose.yaw;
    Msg msg;msg.message=event;msg.detail=YAML::Dump(row);
    if (require_ack_) pending_.emplace(event+":"+key.str(),
      Pending{msg,Clock::now()+std::chrono::milliseconds(1500),std::move(decide)});
    publisher_->publish(msg);
  }
  void expire() {
    bool expired=false;
    for (auto it=pending_.begin();it!=pending_.end();) {
      if (Clock::now()>=it->second.deadline) {
        RCLCPP_ERROR(node_->get_logger(),"Registration timed out: %s",it->first.c_str());
        it=pending_.erase(it);expired=true;
      } else ++it;
    }
    if (expired && !failed_) {failed_=true;nav_.sealAndCancel();}
  }
  void acknowledge(const Msg & msg) {
    expire();
    if (!require_ack_ || failed_) return;
    std::string event;
    if (msg.message=="owned_goal_registered") event="owned_goal_accepted";
    else if (msg.message=="owned_goal_intent_recorded") event="owned_goal_intent";
    else if (msg.message=="owned_goal_rejected_recorded") event="owned_goal_rejected";
    else return;
    try {
      auto row=YAML::Load(msg.detail);
      if (row["schema"].as<std::string>()!="handyman-owned-goal-ack-v1" ||
        row["task_id"].as<std::string>()!=task_) return;
      auto it=pending_.find(event+":"+row["goal_id"].as<std::string>());
      if (it==pending_.end() || row["registration"].as<std::string>()!=it->second.message.detail) return;
      RCLCPP_INFO(node_->get_logger(),"Registration acknowledged: %s",it->first.c_str());
      auto decide=std::move(it->second.decide);pending_.erase(it);
      if (decide) {
        // Persistence can take time. Recheck current map/lease authority at the
        // actual release point, not only when the navigation plan was started.
        bool allowed=false;
        try {allowed=!dispatch_permit_ || dispatch_permit_();} catch (...) {}
        if (!allowed) {failed_=true;nav_.sealAndCancel();}
        decide(allowed);
      }
    } catch (const YAML::Exception &) {return;}
  }
  void tick() {
    expire();
    for (const auto & entry:pending_) publisher_->publish(entry.second.message);
    if (!failed_) return;
    YAML::Node row;row["schema"]="handyman-search-worker-fault-v1";
    row["task_id"]=task_;row["reason"]="goal_registration_timeout";
    Msg msg;msg.message="search_worker_fault";msg.detail=YAML::Dump(row);fault_->publish(msg);
  }
  rclcpp::Node * node_;
  NavigationExecutor & nav_;
  std::string task_,point_,digest_;
  bool require_ack_,failed_=false;
  std::function<bool()> dispatch_permit_;
  std::map<std::string,Pending> pending_;
  rclcpp::Publisher<Msg>::SharedPtr publisher_,fault_;
  rclcpp::Subscription<Msg>::SharedPtr ack_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
