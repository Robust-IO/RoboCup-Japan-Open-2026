#pragma once
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>
#include <handyman_msgs/msg/handyman_msg.hpp>
#include "handyman_rebuild_ros2/navigation_executor.hpp"

namespace handyman_rebuild_ros2 {
// One unique task ID per lifetime. Node/nav must outlive this object.
// Single-threaded executor only. Owner must check dispatchAllowed immediately
// before dispatch, retain this object while goals exist, and retire explicitly.
// This is not map/vision authorization, durable recovery, or a standstill proof.
class SearchSupervisorLease {
public:
  struct Topics { std::string requests, replies, statuses; };
  SearchSupervisorLease(rclcpp::Node * node, NavigationExecutor & nav,
    std::string task, const Topics & topics)
  : node_(node), nav_(nav), task_(std::move(task)),
    deadline_(Clock::now()+std::chrono::seconds(2)) {
    if (!node || task_.size()!=32 || task_==std::string(32,'0') ||
      !std::all_of(task_.begin(),task_.end(),[](char c) {
        return (c>='0' && c<='9') || (c>='a' && c<='f'); }) ||
      topics.requests.empty() || topics.replies.empty() || topics.statuses.empty())
      throw std::invalid_argument("invalid search lease owner/topics");
    request_=node_->create_publisher<Msg>(topics.requests,10);
    status_=node_->create_publisher<Msg>(topics.statuses,10);
    reply_=node_->create_subscription<Msg>(topics.replies,10,[this](const Msg & msg) {receive(msg);});
    timer_=node_->create_wall_timer(std::chrono::milliseconds(100),[this]() {tick();});
  }
  SearchSupervisorLease(const SearchSupervisorLease &)=delete;
  SearchSupervisorLease & operator=(const SearchSupervisorLease &)=delete;
  bool dispatchAllowed() {expire();return ready_ && !failed_ && !retired_;}
  // Only after owner has sealed navigation for explicit cancellation. No reset.
  void retireAfterSeal() {expire();retired_=true;ready_=false;}
  bool failed() const {return failed_;}
private:
  using Clock=std::chrono::steady_clock;
  using Msg=handyman_msgs::msg::HandymanMsg;
  void expire() {
    if (!retired_ && !failed_ && Clock::now()>=deadline_) {
      failed_=true;ready_=false;nav_.sealAndCancel();
      RCLCPP_ERROR(node_->get_logger(),"Supervisor lease expired; executor sealed");
    }
  }
  void receive(const Msg & msg) {
    expire();
    const auto now=Clock::now();
    if (retired_ || failed_ || !pending_ || now>=challenge_deadline_ || msg.message!="supervisor_lease_reply") return;
    try {
      const auto row=YAML::Load(msg.detail);
      if (row["schema"].as<std::string>()!="handyman-supervisor-lease-v1" ||
        row["task_id"].as<std::string>()!=task_ || row["challenge"].as<uint64_t>()!=challenge_) return;
      pending_=false;ready_=true;deadline_=now+std::chrono::seconds(1);
    } catch (const YAML::Exception &) {return;}
  }
  void tick() {
    if (retired_ && !failed_) return;
    expire();
    YAML::Node row;row["task_id"]=task_;
    Msg msg;
    if (failed_) {
      row["schema"]="handyman-search-worker-fault-v1";row["reason"]="supervisor_lease_timeout";
      msg.message="search_worker_fault";msg.detail=YAML::Dump(row);status_->publish(msg);return;
    }
    const auto now=Clock::now();
    if (pending_ && now<challenge_deadline_) return;
    ++challenge_;pending_=true;challenge_deadline_=now+std::chrono::milliseconds(400);
    row["schema"]="handyman-supervisor-lease-v1";row["challenge"]=challenge_;
    msg.message="supervisor_lease_request";msg.detail=YAML::Dump(row);request_->publish(msg);
  }
  rclcpp::Node * node_;
  NavigationExecutor & nav_;
  std::string task_;
  bool ready_=false,failed_=false,pending_=false,retired_=false;
  uint64_t challenge_=0;
  Clock::time_point deadline_,challenge_deadline_{};
  rclcpp::Publisher<Msg>::SharedPtr request_,status_;
  rclcpp::Subscription<Msg>::SharedPtr reply_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
