#pragma once
#include <map>
#include <functional>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace handyman_rebuild_ros2 {
// Humble ClientBase exposes a virtual send_goal_request hook. Keep the original
// request/UUID/callback intact; no ROS library patch and no second UUID generated.
// Single-threaded use, like NavigationExecutor.
class DeferredNavigationClient : public rclcpp_action::Client<nav2_msgs::action::NavigateToPose> {
public:
  using Action=nav2_msgs::action::NavigateToPose;
  using Base=rclcpp_action::Client<Action>;
  using Decision=std::function<void(bool)>;
  using BeforeSend=std::function<void(const rclcpp_action::GoalUUID &,Decision)>;
  using Base::Base;
  BeforeSend before_send;
  void discardPending() {
    auto pending=std::move(pending_);pending_.clear();
    for (auto & entry:pending) reject(entry.second.callback);
  }
  void seal() {sealed_=true;discardPending();}
  static std::shared_ptr<DeferredNavigationClient> create(rclcpp::Node * node,const std::string & name) {
    std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak=node->get_node_waitables_interface();
    auto client=std::shared_ptr<DeferredNavigationClient>(new DeferredNavigationClient(
      node->get_node_base_interface(),node->get_node_graph_interface(),node->get_node_logging_interface(),name),
      [weak](DeferredNavigationClient * ptr) {
        if (auto waitables=weak.lock()) {
          std::shared_ptr<DeferredNavigationClient> borrowed(ptr,[](auto *) {});
          waitables->remove_waitable(borrowed,nullptr);
        }
        delete ptr;
      });
    node->get_node_waitables_interface()->add_waitable(client,nullptr);
    return client;
  }
protected:
  void send_goal_request(std::shared_ptr<void> request,ResponseCallback callback) override {
    if (sealed_) {reject(callback);return;}
    auto typed=std::static_pointer_cast<Action::Impl::SendGoalService::Request>(request);
    const auto id=typed->goal_id.uuid;
    pending_.emplace(id,Pending{request,std::move(callback)});
    auto decide=[this,id](bool permit) {
      auto it=pending_.find(id);
      if (it==pending_.end()) return;
      auto pending=std::move(it->second);pending_.erase(it);
      if (permit && !sealed_) Base::send_goal_request(pending.request,std::move(pending.callback));
      else reject(pending.callback);
    };
    if (before_send) before_send(id,decide);
    else decide(true);
  }
private:
  struct Pending {std::shared_ptr<void> request;ResponseCallback callback;};
  static void reject(const ResponseCallback & callback) {
    auto response=std::make_shared<Action::Impl::SendGoalService::Response>();
    response->accepted=false;callback(response);
  }
  bool sealed_=false;
  std::map<rclcpp_action::GoalUUID,Pending> pending_;
};
}
