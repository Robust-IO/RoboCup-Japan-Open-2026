#pragma once
#include <algorithm>
#include <functional>
#include <yaml-cpp/yaml.h>
#include <handyman_msgs/msg/handyman_msg.hpp>
#include "handyman_rebuild_ros2/navigation_executor.hpp"

namespace handyman_rebuild_ros2 {
// One task / executor lifetime. Use a single-threaded executor. Does not replace
// the per-goal cancellation observer, which must be installed by the owner.
class SearchCancelReporter {
public:
  SearchCancelReporter(rclcpp::Node * node, NavigationExecutor & nav, std::string task,
    const std::string & requests, const std::string & statuses,
    std::function<std::string()> protection_state={})
  : nav_(nav), task_(std::move(task)), protection_state_(std::move(protection_state)) {
    publisher_ = node->create_publisher<handyman_msgs::msg::HandymanMsg>(statuses, 10);
    subscription_ = node->create_subscription<handyman_msgs::msg::HandymanMsg>(
      requests, rclcpp::QoS(1).reliable().transient_local(),
      [this](const handyman_msgs::msg::HandymanMsg & msg) {
        if (msg.message != "search_cancelled") return;
        try {
          auto row = YAML::Load(msg.detail);
          if (row["schema"].as<std::string>() != "handyman-search-request-v1" ||
            row["task_id"].as<std::string>() != task_) return;
          const auto id = row["cancel_id"].as<std::string>();
          if (id.size() != 32 || id == std::string(32, '0') ||
            !std::all_of(id.begin(), id.end(), [](char c) {
              return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); })) return;
          if (!cancel_.empty()) return; // Duplicate does not extend deadline.
          cancel_ = id;
          deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
          nav_.sealAndCancel();
          publish("cancel_received");
        } catch (const YAML::Exception &) { return; }
      });
    timer_ = node->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (cancel_.empty()) return;
      if (terminal_.empty()) {
        const auto state = nav_.cancellationDrainState();
        const auto protection = protection_state_ ? protection_state_() : "cancel_drained";
        // A late terminal must not undo our own deadline failure.
        if (std::chrono::steady_clock::now() >= deadline_ || protection == "cancel_failed" ||
          state == "cancel_failed") terminal_ = "cancel_failed";
        else if (protection == "cancel_drained" && state != "cancel_received") terminal_ = state;
      }
      publish(terminal_.empty() ? "cancel_received" : terminal_);
    });
  }
  bool cancelling() const { return !cancel_.empty(); }
  bool drainedFor(const std::string & id) const {
    return !id.empty() && id==cancel_ && terminal_=="cancel_drained";
  }
private:
  void publish(const std::string & state) {
    YAML::Node row;
    row["schema"] = "handyman-search-cancel-status-v1";
    row["task_id"] = task_; row["cancel_id"] = cancel_; row["state"] = state;
    handyman_msgs::msg::HandymanMsg msg;
    msg.message = "search_cancel_status"; msg.detail = YAML::Dump(row);
    publisher_->publish(msg);
  }
  NavigationExecutor & nav_;
  std::string task_, cancel_, terminal_;
  std::function<std::string()> protection_state_;
  std::chrono::steady_clock::time_point deadline_{};
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr publisher_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
