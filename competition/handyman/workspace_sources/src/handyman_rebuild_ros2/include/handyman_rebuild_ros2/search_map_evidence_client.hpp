#pragma once
#include "handyman_rebuild_ros2/search_execution_guard.hpp"

namespace handyman_rebuild_ros2 {
// Challenge-bound map evidence; single-threaded executor, one task lifetime.
// A verified response is usable for at most one steady-clock second.
class SearchMapEvidenceClient {
public:
  using Guard=SearchExecutionGuard;
  using Msg=handyman_msgs::msg::HandymanMsg;
  using Clock=std::chrono::steady_clock;
  SearchMapEvidenceClient(rclcpp::Node * node,std::string task,std::string point,std::string digest,
    const std::string & requests,const std::string & replies)
  : task_(std::move(task)),point_(std::move(point)),digest_(std::move(digest)) {
    publisher_=node->create_publisher<Msg>(requests,10);
    subscription_=node->create_subscription<Msg>(replies,10,[this](const Msg & msg) {
      expire();
      if (revoked_ || !pending_ || Clock::now()>=response_deadline_ || msg.message!="map_check_reply") return;
      try {
        auto row=YAML::Load(msg.detail);
        if (row["schema"].as<std::string>()!="handyman-map-check-v1" ||
          row["task_id"].as<std::string>()!=task_ || row["point_id"].as<std::string>()!=point_ ||
          row["map_sha256"].as<std::string>()!=digest_ || row["challenge"].as<uint64_t>()!=challenge_) return;
        pending_=false;
        const auto state=row["state"].as<std::string>();
        if (state=="verified") {verified_=true;valid_until_=Clock::now()+std::chrono::seconds(1);}
        else if (state=="revoked" || (verified_ && state=="pending")) revoked_=true;
      } catch (const YAML::Exception &) {return;}
    });
    timer_=node->create_wall_timer(std::chrono::milliseconds(100),[this]() {
      expire();
      if (revoked_ || (pending_ && Clock::now()<response_deadline_)) return;
      ++challenge_;pending_=true;response_deadline_=Clock::now()+std::chrono::milliseconds(400);
      YAML::Node row;row["schema"]="handyman-map-check-v1";row["task_id"]=task_;
      row["point_id"]=point_;row["map_sha256"]=digest_;row["challenge"]=challenge_;
      Msg msg;msg.message="map_check_request";msg.detail=YAML::Dump(row);publisher_->publish(msg);
    });
  }
  SearchMapEvidenceClient(const SearchMapEvidenceClient &)=delete;
  SearchMapEvidenceClient & operator=(const SearchMapEvidenceClient &)=delete;
  Guard::MapEvidence evidence() {
    expire();return {revoked_ ? Guard::MapState::revoked : verified_ ? Guard::MapState::verified : Guard::MapState::pending,
      task_,point_,digest_};
  }
private:
  void expire() {if (verified_ && Clock::now()>=valid_until_) revoked_=true;}
  std::string task_,point_,digest_;
  bool pending_=false,verified_=false,revoked_=false;
  uint64_t challenge_=0;
  Clock::time_point response_deadline_{},valid_until_{};
  rclcpp::Publisher<Msg>::SharedPtr publisher_;
  rclcpp::Subscription<Msg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
