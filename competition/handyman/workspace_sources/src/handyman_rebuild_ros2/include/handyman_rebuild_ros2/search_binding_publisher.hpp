#pragma once
#include <iomanip>
#include <sstream>
#include <handyman_msgs/msg/handyman_msg.hpp>
#include <yaml-cpp/yaml.h>
#include "handyman_rebuild_ros2/navigation_executor.hpp"

namespace handyman_rebuild_ros2 {
// Opt-in telemetry only. Owner supplies task identity and independently pinned map digest.
class SearchBindingPublisher {
public:
  SearchBindingPublisher(rclcpp::Node * node, std::string task, std::string point,
    std::string digest, const std::string & topic)
  : node_(node), task_(std::move(task)), point_(std::move(point)), digest_(std::move(digest)),
    pub_(node->create_publisher<handyman_msgs::msg::HandymanMsg>(topic, 10)) {}

  void accepted(const rclcpp_action::GoalUUID & uuid, std::uint64_t generation,
    const NavigationCandidate & candidate)
  {
    std::ostringstream key;
    for (auto byte : uuid) key << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    YAML::Node row;
    row["schema"]="handyman-search-binding-v1";
    row["stamp_ns"]=node_->now().nanoseconds();
    row["task_id"]=task_; row["point_id"]=point_; row["map_sha256"]=digest_;
    row["goal_id"]=key.str(); row["generation"]=generation;
    row["role"]="final_search_point"; row["frame_id"]="map";
    row["pose"]["x"]=candidate.pose.x;
    row["pose"]["y"]=candidate.pose.y;
    row["pose"]["yaw"]=candidate.pose.yaw;
    handyman_msgs::msg::HandymanMsg msg;
    msg.message="search_goal_accepted"; msg.detail=YAML::Dump(row);
    pub_->publish(msg);
  }
private:
  rclcpp::Node * node_;
  std::string task_, point_, digest_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr pub_;
};
}
