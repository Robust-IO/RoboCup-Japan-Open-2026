#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <map>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "handyman_rebuild_ros2/environment_config.hpp"
#include "handyman_rebuild_ros2/navigation_plan.hpp"
#include "handyman_rebuild_ros2/deferred_navigation_client.hpp"

namespace handyman_rebuild_ros2
{

enum class NavigationTarget
{
  kRoom,
  kDestination,
  kSearchPoint,
};

struct NavigationOutcome
{
  bool success{false};
  NavigationTarget target{NavigationTarget::kRoom};
  std::size_t attempts{0};
  Pose2D robot_pose{};
  std::string reason;
};

struct NavigationSettings
{
  std::size_t maximum_attempts{3};
  double goal_timeout_sec{60.0};
  double server_wait_timeout_sec{15.0};
  double room_boundary_tolerance_m{0.20};
  double destination_tolerance_m{0.75};
  std::string map_frame{"map"};
  std::string robot_frame{"base_footprint"};
  std::string action_name{"navigate_to_pose"};
  // Search-only final-goal override; route waypoints retain normal navigation.
  std::string search_behavior_tree{};
};

class NavigationExecutor
{
public:
  using CompletionCallback = std::function<void(const NavigationOutcome &)>;
  // Called only for an accepted FINAL search-point goal, never route waypoints.
  // Consumer must not reenter this executor from the callback.
  using SearchGoalCallback = std::function<void(
    const rclcpp_action::GoalUUID &, std::uint64_t, const NavigationCandidate &)>;
  bool navigateToSearchPoint(
    const std::string & environment, const std::string & room, std::size_t index,
    SearchGoalCallback accepted, CompletionCallback completion);

  NavigationExecutor(
    rclcpp::Node * node,
    const EnvironmentCatalog * catalog,
    NavigationSettings settings = {});

  bool navigateToRoom(
    const std::string & environment,
    const std::string & room,
    CompletionCallback completion);

  bool navigateToDestination(
    const std::string & environment,
    const std::string & destination,
    const std::string & destination_room,
    CompletionCallback completion);

  void cancel();
  // Permanently seal this executor against new dispatch and cancel all owned goals.
  // Poll only from the same single-threaded callback context as navigation methods.
  void sealAndCancel();
  std::string cancellationDrainState() const;
  using CancellationCallback = std::function<void(const rclcpp_action::GoalUUID &, const std::string &)>;
  void setCancellationObserver(CancellationCallback callback);
  // Reports every accepted search action, including route waypoints and stale
  // late acceptance. One owner/task per observer lifetime; do not reenter here.
  using OwnedGoalCallback = std::function<void(const rclcpp_action::GoalUUID &,
    std::uint64_t, const Pose2D &, bool)>;
  void setOwnedGoalObserver(OwnedGoalCallback callback) { owned_goal_callback_ = std::move(callback); }
  using DispatchIntentCallback = std::function<void(const rclcpp_action::GoalUUID &,
    std::uint64_t,const Pose2D &,bool,DeferredNavigationClient::Decision)>;
  void setDispatchIntentObserver(DispatchIntentCallback callback) {dispatch_intent_callback_=std::move(callback);}
  void setRejectedGoalObserver(OwnedGoalCallback callback) {rejected_goal_callback_=std::move(callback);}
  bool active() const noexcept;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  bool begin(
    const EnvironmentConfig & environment,
    NavigationPlan plan,
    NavigationTarget target,
    CompletionCallback completion);
  void waitForServer();
  void sendCurrentGoal();
  void handleGoalRejected(std::uint64_t token);
  void failAttempt(std::uint64_t token, const std::string & reason);
  void verifyResult(std::uint64_t token);
  void finish(bool success, const std::string & reason, const Pose2D & pose = {});
  void clearCostmaps();
  std::optional<Pose2D> lookupRobotPose() const;
  void cancelTimers();
  void cancelGoalTracked(const GoalHandle::SharedPtr & handle);
  CancellationCallback cancellation_callback_;
  std::map<rclcpp_action::GoalUUID, std::chrono::steady_clock::time_point> cancelling_goals_;
  rclcpp::TimerBase::SharedPtr cancellation_timer_;
  std::size_t pending_goal_responses_{0};
  std::map<rclcpp_action::GoalUUID, GoalHandle::SharedPtr> owned_goals_;
  bool sealed_{false};
  bool cancellation_fault_{false};
  OwnedGoalCallback owned_goal_callback_;
  OwnedGoalCallback rejected_goal_callback_;
  DispatchIntentCallback dispatch_intent_callback_;

  rclcpp::Node * node_;
  const EnvironmentCatalog * catalog_;
  NavigationSettings settings_;
  std::shared_ptr<DeferredNavigationClient> action_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_;
  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<NavigationPlan> plan_;
  const EnvironmentConfig * environment_{nullptr};
  NavigationTarget target_{NavigationTarget::kRoom};
  CompletionCallback completion_;
  SearchGoalCallback search_goal_callback_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr server_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::chrono::steady_clock::time_point server_wait_started_;
  std::uint64_t generation_{0};
  std::size_t route_waypoint_index_{0};
};

}  // namespace handyman_rebuild_ros2
