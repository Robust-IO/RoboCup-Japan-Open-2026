#include "handyman_rebuild_ros2/navigation_executor.hpp"

#include <cmath>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace handyman_rebuild_ros2
{

NavigationExecutor::NavigationExecutor(
  rclcpp::Node * node,
  const EnvironmentCatalog * catalog,
  NavigationSettings settings)
: node_(node),
  catalog_(catalog),
  settings_(std::move(settings)),
  action_client_(DeferredNavigationClient::create(node, settings_.action_name)),
  clear_global_costmap_(node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap")),
  clear_local_costmap_(node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap")),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
{
}

bool NavigationExecutor::navigateToRoom(
  const std::string & environment,
  const std::string & room,
  CompletionCallback completion)
{
  if (sealed_) return false;
  const EnvironmentConfig * config = catalog_ == nullptr ? nullptr : catalog_->find(environment);
  if (config == nullptr) {
    return false;
  }
  const auto robot_pose = lookupRobotPose();
  const auto room_config = config->rooms.find(room);
  if (robot_pose && room_config != config->rooms.end() &&
    isPoseInsideRoom(room_config->second, *robot_pose, settings_.room_boundary_tolerance_m))
  {
    NavigationOutcome outcome;
    outcome.success = true;
    outcome.target = NavigationTarget::kRoom;
    outcome.robot_pose = *robot_pose;
    outcome.reason = "Robot is already inside room " + room;
    completion(outcome);
    return true;
  }
  return begin(
    *config,
    NavigationPlan::forRoom(*config, room, settings_.maximum_attempts, robot_pose),
    NavigationTarget::kRoom,
    std::move(completion));
}

bool NavigationExecutor::navigateToDestination(
  const std::string & environment,
  const std::string & destination,
  const std::string & destination_room,
  CompletionCallback completion)
{
  const EnvironmentConfig * config = catalog_ == nullptr ? nullptr : catalog_->find(environment);
  if (config == nullptr) {
    return false;
  }
  const auto robot_pose = lookupRobotPose();
  return begin(
    *config,
    NavigationPlan::forDestination(
      *config, destination, destination_room, settings_.maximum_attempts, robot_pose),
    NavigationTarget::kDestination,
    std::move(completion));
}

bool NavigationExecutor::navigateToSearchPoint(
  const std::string & environment, const std::string & room, std::size_t index,
  SearchGoalCallback accepted, CompletionCallback completion)
{
  const auto * config = catalog_ == nullptr ? nullptr : catalog_->find(environment);
  if (config == nullptr || !accepted) {
    return false;
  }
  // begin may submit asynchronously; callbacks run after this method returns.
  const bool started = begin(*config,
    NavigationPlan::forSearchPoint(*config, room, index, settings_.maximum_attempts,
      lookupRobotPose()), NavigationTarget::kSearchPoint, std::move(completion));
  if (started) {
    search_goal_callback_ = std::move(accepted);
  }
  return started;
}

bool NavigationExecutor::begin(
  const EnvironmentConfig & environment,
  NavigationPlan plan,
  NavigationTarget target,
  CompletionCallback completion)
{
  if (sealed_) return false;
  cancel();
  if (!plan.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot create navigation plan: %s", plan.error().c_str());
    return false;
  }
  environment_ = &environment;
  plan_ = std::move(plan);
  target_ = target;
  completion_ = std::move(completion);
  route_waypoint_index_ = 0;
  server_wait_started_ = std::chrono::steady_clock::now();
  waitForServer();
  return true;
}

void NavigationExecutor::waitForServer()
{
  if (!active()) {
    return;
  }
  if (action_client_->action_server_is_ready()) {
    if (server_timer_) {
      server_timer_->cancel();
      server_timer_.reset();
    }
    sendCurrentGoal();
    return;
  }
  const double waited = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - server_wait_started_).count();
  if (waited >= settings_.server_wait_timeout_sec) {
    finish(false, "Nav2 action server was not available before timeout");
    return;
  }
  if (!server_timer_) {
    server_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(250), [this]() {waitForServer();});
  }
}

void NavigationExecutor::sendCurrentGoal()
{
  if (sealed_ || !active()) return;
  const NavigationCandidate * candidate = plan_ ? plan_->current() : nullptr;
  if (candidate == nullptr) {
    finish(false, "Navigation plan has no remaining candidate");
    return;
  }
  const std::uint64_t token = ++generation_;
  const bool is_route_waypoint = route_waypoint_index_ < candidate->route_waypoints.size();
  const Pose2D & target_pose = is_route_waypoint ?
    candidate->route_waypoints[route_waypoint_index_] : candidate->pose;
  NavigateToPose::Goal goal;
  if (target_ == NavigationTarget::kSearchPoint && !is_route_waypoint &&
    !settings_.search_behavior_tree.empty())
  {
    const auto current_pose = lookupRobotPose();
    if (!current_pose) {
      const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now()-server_wait_started_).count();
      if (waited >= settings_.server_wait_timeout_sec) {
        finish(false, "Search turn TF unavailable before startup deadline");
        return;
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "Waiting for first search-turn TF before dispatch");
      retry_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this]() {
          retry_timer_->cancel();
          retry_timer_.reset();
          sendCurrentGoal();
        });
      return;
    }
    const double distance = std::hypot(current_pose->x-target_pose.x, current_pose->y-target_pose.y);
    if (distance <= 0.14) {
      goal.behavior_tree = settings_.search_behavior_tree;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Search ring position drift %.3f m: using normal navigation to recenter", distance);
    }
  }
  goal.pose.header.frame_id = settings_.map_frame;
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = target_pose.x;
  goal.pose.pose.position.y = target_pose.y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, target_pose.yaw);
  goal.pose.pose.orientation = tf2::toMsg(orientation);

  typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  const bool search_owned = target_ == NavigationTarget::kSearchPoint;
  auto identity=std::make_shared<rclcpp_action::GoalUUID>();
  action_client_->before_send=[this,token,target_pose,is_route_waypoint,search_owned,identity](const auto & id,auto decide) {
    *identity=id;
    if (search_owned && dispatch_intent_callback_) {
      dispatch_intent_callback_(id,token,target_pose,is_route_waypoint,
        [this,token,decide](bool permit) {decide(permit && !sealed_ && active() && token==generation_);});
    } else decide(true);
  };
  options.goal_response_callback = [this, token, is_route_waypoint, target_pose, search_owned,identity](const GoalHandle::SharedPtr & handle) {
      --pending_goal_responses_;
      if (!handle && search_owned && rejected_goal_callback_) {
        rejected_goal_callback_(*identity,token,target_pose,is_route_waypoint);
      }
      if (handle) owned_goals_[handle->get_goal_id()] = handle;
      if (handle && search_owned && owned_goal_callback_) {
        owned_goal_callback_(handle->get_goal_id(), token, target_pose, is_route_waypoint);
      }
      if (!active() || token != generation_) {
        // A cancellation may precede the action acceptance response. Retire that
        // exact late goal; never cancel unrelated goals on the server.
        if (handle) {
          cancelGoalTracked(handle);
        }
        return;
      }
      if (!handle) {
        handleGoalRejected(token);
        return;
      }
      goal_handle_ = handle;
      if (target_ == NavigationTarget::kSearchPoint && !is_route_waypoint &&
        search_goal_callback_ && plan_ && plan_->current())
      {
        search_goal_callback_(handle->get_goal_id(), token, *plan_->current());
      }
    };
  options.feedback_callback = [this, token](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (active() && token == generation_) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "Navigation attempt %zu/%zu: %.2f m remaining",
          plan_->attemptNumber(), plan_->maximumAttempts(), feedback->distance_remaining);
      }
    };
  options.result_callback = [this, token](const GoalHandle::WrappedResult & result) {
      owned_goals_.erase(result.goal_id);
      if (cancelling_goals_.erase(result.goal_id) && cancellation_callback_) {
        if (result.code != rclcpp_action::ResultCode::CANCELED) cancellation_fault_ = true;
        cancellation_callback_(result.goal_id,
          result.code == rclcpp_action::ResultCode::CANCELED ? "cancel_confirmed" : "goal_ended_otherwise");
      }
      if (!active() || token != generation_) {
        return;
      }
      goal_handle_.reset();
      if (goal_timer_) {
        goal_timer_->cancel();
        goal_timer_.reset();
      }
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        const NavigationCandidate * current = plan_ ? plan_->current() : nullptr;
        if (current && route_waypoint_index_ < current->route_waypoints.size()) {
          ++route_waypoint_index_;
          // Do not submit a new action from inside Nav2's result callback.  A
          // short delay lets the server finish retiring the previous goal and
          // prevents the next waypoint being mistaken for a preemption.
          retry_timer_ = node_->create_wall_timer(std::chrono::milliseconds(250), [this]() {
              retry_timer_->cancel();
              retry_timer_.reset();
              sendCurrentGoal();
            });
        } else {
          verifyResult(token);
        }
      } else {
        failAttempt(token, "Nav2 goal ended with result code " +
          std::to_string(static_cast<int>(result.code)));
      }
    };

  RCLCPP_INFO(
    node_->get_logger(),
    "Sending navigation attempt %zu/%zu %s %zu to (%.3f, %.3f, %.3f)",
    plan_->attemptNumber(), plan_->maximumAttempts(),
    is_route_waypoint ? "via waypoint" : "to candidate",
    is_route_waypoint ? route_waypoint_index_ + 1 : candidate->source_index,
    target_pose.x, target_pose.y, target_pose.yaw);
  ++pending_goal_responses_;
  action_client_->async_send_goal(goal, options);
  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(settings_.goal_timeout_sec));
  goal_timer_ = node_->create_wall_timer(timeout, [this, token]() {
      if (!active() || token != generation_) {
        return;
      }
      if (goal_handle_) {
        cancelGoalTracked(goal_handle_);
      } else {
        // Acceptance can still be in flight. Seal and cancel exact late handles;
        // never cancel unrelated goals merely because our handle is unavailable.
        sealAndCancel();
        return;
      }
      failAttempt(token, "Navigation goal timed out");
    });
}

void NavigationExecutor::handleGoalRejected(std::uint64_t token)
{
  if (!active() || token != generation_) {
    return;
  }
  ++generation_;
  if (goal_timer_) {
    goal_timer_->cancel();
    goal_timer_.reset();
  }
  const double waited = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - server_wait_started_).count();
  if (waited >= settings_.server_wait_timeout_sec) {
    // Restore the token expected by failAttempt. At this point a rejection is
    // treated as a real attempt failure rather than a lifecycle warm-up race.
    failAttempt(generation_, "Nav2 kept rejecting goals before the startup timeout");
    return;
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "Nav2 rejected the goal while its lifecycle is starting; retrying the same candidate");
  retry_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      retry_timer_->cancel();
      retry_timer_.reset();
      sendCurrentGoal();
    });
}

void NavigationExecutor::verifyResult(std::uint64_t token)
{
  const NavigationCandidate * candidate = plan_ ? plan_->current() : nullptr;
  const auto robot_pose = lookupRobotPose();
  if (candidate == nullptr || !robot_pose || environment_ == nullptr) {
    failAttempt(token, "Nav2 succeeded but robot pose was unavailable for verification");
    return;
  }
  const auto room = environment_->rooms.find(candidate->room);
  if (room == environment_->rooms.end() ||
    !isPoseInsideRoom(room->second, *robot_pose, settings_.room_boundary_tolerance_m))
  {
    failAttempt(token, "Robot pose is outside the expected room after navigation");
    return;
  }
  if (target_ == NavigationTarget::kDestination &&
    std::hypot(robot_pose->x - candidate->pose.x, robot_pose->y - candidate->pose.y) >
    settings_.destination_tolerance_m)
  {
    failAttempt(token, "Robot stopped too far from the destination candidate");
    return;
  }
  if (target_ == NavigationTarget::kSearchPoint &&
    (std::hypot(robot_pose->x - candidate->pose.x, robot_pose->y - candidate->pose.y) > 0.15 ||
    std::abs(std::atan2(std::sin(robot_pose->yaw - candidate->pose.yaw),
    std::cos(robot_pose->yaw - candidate->pose.yaw))) > 0.17453292519943295))
  {
    failAttempt(token, "Robot stopped outside search-point tolerance");
    return;
  }
  finish(true, "Navigation and region verification succeeded", *robot_pose);
}

void NavigationExecutor::failAttempt(std::uint64_t token, const std::string & reason)
{
  if (!active() || token != generation_) {
    return;
  }
  ++generation_;
  goal_handle_.reset();
  if (goal_timer_) {
    goal_timer_->cancel();
    goal_timer_.reset();
  }
  RCLCPP_WARN(node_->get_logger(), "Navigation attempt failed: %s", reason.c_str());
  clearCostmaps();
  // Keep the index of the current route waypoint.  Replaying already reached
  // waypoints can send the robot backwards through a doorway during recovery.
  if (!plan_->advance()) {
    finish(false, reason + "; all navigation attempts were exhausted");
    return;
  }
  retry_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      retry_timer_->cancel();
      retry_timer_.reset();
      sendCurrentGoal();
    });
}

void NavigationExecutor::clearCostmaps()
{
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  if (clear_global_costmap_->service_is_ready()) {
    clear_global_costmap_->async_send_request(request);
  }
  if (clear_local_costmap_->service_is_ready()) {
    clear_local_costmap_->async_send_request(request);
  }
}

std::optional<Pose2D> NavigationExecutor::lookupRobotPose() const
{
  try {
    const auto transform = tf_buffer_.lookupTransform(
      settings_.map_frame, settings_.robot_frame, tf2::TimePointZero);
    return Pose2D{
      transform.transform.translation.x,
      transform.transform.translation.y,
      tf2::getYaw(transform.transform.rotation)};
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

void NavigationExecutor::finish(
  bool success,
  const std::string & reason,
  const Pose2D & pose)
{
  NavigationOutcome outcome;
  outcome.success = success;
  outcome.target = target_;
  outcome.attempts = plan_ ? plan_->attemptNumber() : 0;
  outcome.robot_pose = pose;
  outcome.reason = reason;
  auto completion = std::move(completion_);
  search_goal_callback_ = {};
  ++generation_;
  cancelTimers();
  goal_handle_.reset();
  plan_.reset();
  environment_ = nullptr;
  route_waypoint_index_ = 0;
  if (completion) {
    completion(outcome);
  }
}

void NavigationExecutor::cancelTimers()
{
  for (auto * timer : {&server_timer_, &goal_timer_, &retry_timer_}) {
    if (*timer) {
      (*timer)->cancel();
      timer->reset();
    }
  }
}

void NavigationExecutor::cancel()
{
  ++generation_;
  action_client_->discardPending();
  if (goal_handle_) {
    cancelGoalTracked(goal_handle_);
  }
  cancelTimers();
  goal_handle_.reset();
  plan_.reset();
  environment_ = nullptr;
  route_waypoint_index_ = 0;
  completion_ = {};
  search_goal_callback_ = {};
}

bool NavigationExecutor::active() const noexcept {return plan_.has_value();}

void NavigationExecutor::sealAndCancel()
{
  sealed_ = true;
  cancel();
  action_client_->seal();
  // Includes older accepted goals whose timeout/retry detached goal_handle_.
  for (const auto & entry : owned_goals_) cancelGoalTracked(entry.second);
}

std::string NavigationExecutor::cancellationDrainState() const
{
  if (cancellation_fault_) return "cancel_failed";
  if (!sealed_ || active() || pending_goal_responses_ || !owned_goals_.empty() ||
    !cancelling_goals_.empty()) return "cancel_received";
  return "cancel_drained";
}

void NavigationExecutor::setCancellationObserver(CancellationCallback callback)
{
  cancellation_callback_ = std::move(callback);
}

void NavigationExecutor::cancelGoalTracked(const GoalHandle::SharedPtr & handle)
{
  const auto id = handle->get_goal_id();
  if (!cancellation_callback_) {
    action_client_->async_cancel_goal(handle);
    return;
  }
  if (cancelling_goals_.count(id)) return;
  cancelling_goals_[id] = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  cancellation_callback_(id, "cancel_requested");
  action_client_->async_cancel_goal(handle,
    [this,id](action_msgs::srv::CancelGoal::Response::SharedPtr response) {
      if (!cancelling_goals_.count(id) || !cancellation_callback_) return;
      bool accepted = response && response->return_code == 0;
      bool matching = false;
      if (response) for (const auto & goal : response->goals_canceling) {
        if (goal.goal_id.uuid == id) matching = true;
      }
      if (accepted && matching) {
        cancellation_callback_(id, "cancel_accepted_waiting_terminal");
      } else {
        cancellation_fault_ = true;
        cancelling_goals_.erase(id);
        cancellation_callback_(id, "cancel_rejected_or_not_acknowledged");
      }
    });
  if (!cancellation_timer_) {
    cancellation_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const auto now = std::chrono::steady_clock::now();
      for (auto it = cancelling_goals_.begin(); it != cancelling_goals_.end();) {
        if (now >= it->second) {
          cancellation_fault_ = true;
          auto id = it->first;
          it = cancelling_goals_.erase(it);
          if (cancellation_callback_) cancellation_callback_(id, "cancel_confirmation_timeout");
        } else {
          ++it;
        }
      }
    });
  }
}

}  // namespace handyman_rebuild_ros2
