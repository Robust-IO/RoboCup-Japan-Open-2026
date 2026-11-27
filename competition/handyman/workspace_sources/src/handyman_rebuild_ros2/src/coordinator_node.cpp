#include "handyman_rebuild_ros2/coordinator_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <iomanip>
#include <random>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace handyman_rebuild_ros2
{

CoordinatorNode::CoordinatorNode(const rclcpp::NodeOptions & options)
: Node("handyman_coordinator", options)
{
    simulate_modules_ = declare_parameter<bool>("simulate_modules", false);
    simulation_step_ms_ = declare_parameter<int>("simulation_step_ms", 100);

    const std::string config_directory =
      ament_index_cpp::get_package_share_directory("handyman_rebuild_ros2") + "/config";
    std::string config_error;
    NameAliases aliases;
    if (aliases.loadFromFile(config_directory + "/name_aliases.yaml", config_error)) {
      instruction_parser_ = std::make_unique<RuleBasedInstructionParser>(std::move(aliases));
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to load name aliases: %s", config_error.c_str());
    }
    if (!environment_catalog_.loadFromFile(
        config_directory + "/environments.yaml", config_error))
    {
      RCLCPP_ERROR(get_logger(), "Failed to load environments: %s", config_error.c_str());
    }
    NavigationSettings navigation_settings;
    try {
      const YAML::Node recovery = YAML::LoadFile(config_directory + "/recovery.yaml");
      navigation_settings.maximum_attempts =
        recovery["retries"]["navigation"].as<std::size_t>();
      navigation_settings.goal_timeout_sec =
        recovery["timeouts_sec"]["navigation"].as<double>();
    } catch (const YAML::Exception & exception) {
      RCLCPP_WARN(
        get_logger(), "Could not load navigation recovery settings; using defaults: %s",
        exception.what());
    }
    const int maximum_attempts = declare_parameter<int>(
      "navigation.maximum_attempts", static_cast<int>(navigation_settings.maximum_attempts));
    navigation_settings.maximum_attempts = static_cast<std::size_t>(std::max(1, maximum_attempts));
    navigation_settings.goal_timeout_sec = declare_parameter<double>(
      "navigation.goal_timeout_sec", navigation_settings.goal_timeout_sec);
    navigation_settings.goal_timeout_sec = std::max(0.1, navigation_settings.goal_timeout_sec);
    navigation_settings.server_wait_timeout_sec = declare_parameter<double>(
      "navigation.server_wait_timeout_sec", navigation_settings.server_wait_timeout_sec);
    navigation_settings.server_wait_timeout_sec = std::max(
      0.1, navigation_settings.server_wait_timeout_sec);
    navigation_settings.room_boundary_tolerance_m = declare_parameter<double>(
      "navigation.room_boundary_tolerance_m", navigation_settings.room_boundary_tolerance_m);
    navigation_settings.destination_tolerance_m = declare_parameter<double>(
      "navigation.destination_tolerance_m", navigation_settings.destination_tolerance_m);
    navigation_settings.map_frame = declare_parameter<std::string>(
      "navigation.map_frame", navigation_settings.map_frame);
    navigation_settings.robot_frame = declare_parameter<std::string>(
      "navigation.robot_frame", navigation_settings.robot_frame);
    navigation_settings.action_name = declare_parameter<std::string>(
      "navigation.action_name", navigation_settings.action_name);
    navigation_executor_ = std::make_unique<NavigationExecutor>(
      this, &environment_catalog_, navigation_settings);
    navigation_executor_->setCancellationObserver([this](const auto & uuid,const std::string & state) {
      std::ostringstream id;
      for (auto byte:uuid) id << std::hex << std::setw(2) << std::setfill('0') << int(byte);
      cancellation_barrier_.event(id.str(),state);
      RCLCPP_WARN(get_logger(),"Navigation cancellation %s: %s; dispatch_blocked=%d",
        id.str().c_str(),state.c_str(),cancellation_barrier_.blocked());
    });
    using handyman_msgs::msg::HandymanMsg;
    search_requests_enabled_ = declare_parameter<bool>("search.publish_requests", false);
    if (search_requests_enabled_ && simulate_modules_) {
      throw std::invalid_argument("search.publish_requests cannot be combined with simulate_modules");
    }
    if (search_requests_enabled_) {
      search_request_publisher_ = create_publisher<HandymanMsg>(
        "/handyman/search/request", rclcpp::QoS(1).reliable().transient_local());
      search_status_subscription_ = create_subscription<HandymanMsg>(
        "/handyman/search/execution_status", rclcpp::QoS(10).reliable(),
        [this](const HandymanMsg & msg) {
          if (msg.message == "search_stop_requested") {
            try {
              const auto row=YAML::Load(msg.detail);
              if (row["schema"].as<std::string>()=="handyman-search-stop-v1" &&
                !search_request_id_.empty() && row["task_id"].as<std::string>()==search_request_id_ &&
                row["reason"].as<std::string>()=="observation_terminal") {
                invalidateSearchRequest("search_stop:observation_terminal");
              }
            } catch (const YAML::Exception &) {}
            return;
          }
          if (msg.message == "search_worker_fault") {
            try {
              const auto row = YAML::Load(msg.detail);
              if (row["schema"].as<std::string>() != "handyman-search-worker-fault-v1" ||
                search_request_id_.empty() || row["task_id"].as<std::string>() != search_request_id_) return;
              const auto reason = row["reason"].as<std::string>();
              if (reason != "worker_exited_without_retirement" && reason != "worker_lifetime_exceeded" &&
                reason != "goal_registration_timeout" && reason != "supervisor_lease_timeout" &&
                reason != "map_verification_revoked" && reason != "search_observer_failed" &&
                reason != "search_runtime_failed") return;
              // Latch BEFORE publishing cancellation: racing drain reports cannot
              // make an unexpected worker failure look like a healthy retirement.
              search_cancellation_barrier_.latchFault();
              invalidateSearchRequest("worker_fault:" + reason);
              RCLCPP_ERROR(get_logger(), "Search worker fault: %s; cancellation requested; dispatch locked",
                reason.c_str());
            } catch (const YAML::Exception &) {
              RCLCPP_WARN(get_logger(), "Ignored malformed search worker fault");
            }
            return;
          }
          if (msg.message != "search_cancel_status") return;
          try {
            const auto row = YAML::Load(msg.detail);
            if (row["schema"].as<std::string>() != "handyman-search-cancel-status-v1") return;
            const bool accepted = search_cancellation_barrier_.event(
              row["task_id"].as<std::string>(), row["cancel_id"].as<std::string>(),
              row["state"].as<std::string>());
            RCLCPP_INFO(get_logger(), "External search cancellation accepted=%d blocked=%d",
              accepted, dispatchBlocked());
          } catch (const YAML::Exception &) {
            RCLCPP_WARN(get_logger(), "Ignored malformed search cancellation status");
          }
        });
    }
    publisher_ = create_publisher<HandymanMsg>(std::string(protocol::kToModeratorTopic), 10);
    subscription_ = create_subscription<HandymanMsg>(
      std::string(protocol::kToRobotTopic), 10,
      [this](const HandymanMsg::ConstSharedPtr message) { onMessage(*message); });

    state_machine_.bootCompleted();
    RCLCPP_INFO(get_logger(), "Handyman coordinator scaffold is waiting for Environment and Are_you_ready?");
    if (simulate_modules_) {
      RCLCPP_WARN(get_logger(), "Module simulation is enabled; no robot commands will be sent");
    }
  }

void CoordinatorNode::stopSimulation()
{
    if (simulation_timer_) {
      simulation_timer_->cancel();
      simulation_timer_.reset();
    }
  }

void CoordinatorNode::startSimulation()
{
    if (!simulate_modules_) {
      return;
    }
    stopSimulation();
    simulation_step_ = 1;
    const auto period = std::chrono::milliseconds(std::max(1, simulation_step_ms_));
    simulation_timer_ = create_wall_timer(period, [this]() { runSimulationStep(); });
  }

void CoordinatorNode::runSimulationStep()
{
    bool transitioned = false;
    switch (simulation_step_++) {
      case 0:
        transitioned = state_machine_.parsingSucceeded();
        break;
      case 1:
        transitioned = state_machine_.roomNavigationSucceeded();
        break;
      case 2:
        transitioned = state_machine_.roomVerified();
        if (transitioned) {
          publishEvent(protocol::CompetitionEvent::kRoomReached);
        }
        break;
      case 3:
        transitioned = state_machine_.objectLocated();
        break;
      case 4:
        transitioned = state_machine_.approachSucceeded();
        break;
      case 5:
        transitioned = state_machine_.graspSucceeded();
        break;
      case 6:
        transitioned = state_machine_.graspVerified();
        if (transitioned) {
          publishEvent(protocol::CompetitionEvent::kObjectGrasped);
        }
        break;
      case 7:
        transitioned = state_machine_.destinationReached();
        break;
      case 8:
        transitioned = state_machine_.placementSucceeded();
        break;
      case 9:
        transitioned = state_machine_.placementVerified();
        if (transitioned) {
          publishEvent(protocol::CompetitionEvent::kTaskFinished);
        }
        stopSimulation();
        break;
      default:
        stopSimulation();
        return;
    }
    if (!transitioned) {
      RCLCPP_ERROR(get_logger(), "Simulated module result was invalid at step %d", simulation_step_ - 1);
      stopSimulation();
    }
  }

bool CoordinatorNode::parseCurrentInstruction(const std::string & instruction)
{
    if (!instruction_parser_) {
      state_machine_.parsingFailed();
      RCLCPP_ERROR(get_logger(), "Instruction parser is unavailable");
      return false;
    }
    HandymanTask parsed_task = state_machine_.task();
    const auto result = instruction_parser_->parse(instruction, parsed_task);
    if (!result.success) {
      state_machine_.parsingFailed();
      RCLCPP_WARN(get_logger(), "Instruction rejected: %s", result.reason.c_str());
      return false;
    }
    std::string environment_error;
    if (!environment_catalog_.resolveTask(parsed_task, environment_error)) {
      state_machine_.parsingFailed();
      RCLCPP_WARN(get_logger(), "Instruction does not match environment: %s", environment_error.c_str());
      return false;
    }
    if (!state_machine_.parsingSucceeded(parsed_task)) {
      RCLCPP_ERROR(get_logger(), "Parsed instruction could not advance the state machine");
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "Parsed task: pickup_room=%s object=%s destination_room=%s destination=%s avatar=%s",
      parsed_task.pickup_room.c_str(), parsed_task.target_object.c_str(),
      parsed_task.destination_room.c_str(), parsed_task.destination.c_str(),
      parsed_task.destination_is_avatar ? "true" : "false");
    return true;
  }

void CoordinatorNode::startRoomNavigation()
{
    if (dispatchBlocked()) {
      RCLCPP_ERROR(get_logger(),"Navigation dispatch blocked: cancellation not confirmed");
      return;
    }
    const HandymanTask task = state_machine_.task();
    const bool started = navigation_executor_ && navigation_executor_->navigateToRoom(
      task.environment, task.pickup_room,
      [this](const NavigationOutcome & outcome) {handleNavigationOutcome(outcome);});
    if (!started) {
      RCLCPP_ERROR(get_logger(), "Could not start room navigation");
      if (state_machine_.giveUp()) {
        publishEvent(protocol::CompetitionEvent::kGiveUp);
      }
    }
  }

void CoordinatorNode::publishSearchRequest()
{
  if (dispatchBlocked()) return;
  if (!search_requests_enabled_ || !search_request_id_.empty()) return;
  const auto & task = state_machine_.task();
  const auto * environment = environment_catalog_.find(task.environment);
  if (!environment) return;
  const auto room = environment->rooms.find(task.pickup_room);
  if (room == environment->rooms.end() || room->second.search_points.empty()) return;
  std::random_device random;
  std::ostringstream id;
  for (int i=0; i<4; ++i) id << std::hex << std::setfill('0') << std::setw(8) << random();
  search_request_id_ = id.str();
  YAML::Node row;
  row["schema"]="handyman-search-request-v1";
  row["task_id"]=search_request_id_;
  row["stamp_ns"]=now().nanoseconds();
  row["environment"]=task.environment;
  row["layout"]=environment->internal_name;
  row["room"]=task.pickup_room;
  row["target"]=task.target_object;
  row["frame_id"]="map";
  row["map_uri"]=environment->map;
  row["requires_map_verification"]=true;
  row["actionable"]=false;
  for (std::size_t index=0; index<room->second.search_points.size(); ++index) {
    const auto & pose=room->second.search_points[index];
    YAML::Node point;
    point["id"]=environment->internal_name+"/"+task.pickup_room+"/"+std::to_string(index);
    point["pose"]["x"]=pose.x; point["pose"]["y"]=pose.y; point["pose"]["yaw"]=pose.yaw;
    row["points"].push_back(point);
  }
  handyman_msgs::msg::HandymanMsg msg;
  msg.message="search_requested"; msg.detail=YAML::Dump(row);
  search_request_publisher_->publish(msg);
}

void CoordinatorNode::invalidateSearchRequest(const std::string & reason)
{
  if (!search_requests_enabled_ || search_request_id_.empty()) return;
  YAML::Node row;
  row["schema"]="handyman-search-request-v1";
  row["task_id"]=search_request_id_; row["stamp_ns"]=now().nanoseconds();
  row["reason"]=reason; row["actionable"]=false;
  std::random_device random;
  std::ostringstream cancellation;
  for (int i=0; i<4; ++i) cancellation << std::hex << std::setfill('0') << std::setw(8) << random();
  row["cancel_id"] = cancellation.str();
  search_cancellation_barrier_.begin(search_request_id_, cancellation.str());
  handyman_msgs::msg::HandymanMsg msg;
  msg.message="search_cancelled"; msg.detail=YAML::Dump(row);
  search_request_publisher_->publish(msg);
  search_request_id_.clear();
}

void CoordinatorNode::stopNavigation()
{
  if (!navigation_executor_) return;
  if (navigation_executor_->active()) cancellation_barrier_.begin();
  navigation_executor_->cancel();
}

bool CoordinatorNode::dispatchBlocked()
{
  return search_cancellation_barrier_.blocked() || cancellation_barrier_.blocked();
}

void CoordinatorNode::handleNavigationOutcome(const NavigationOutcome & outcome)
{
    if (!outcome.success) {
      RCLCPP_ERROR(
        get_logger(), "Navigation failed after %zu attempt(s): %s",
        outcome.attempts, outcome.reason.c_str());
      if (state_machine_.giveUp()) {
        publishEvent(protocol::CompetitionEvent::kGiveUp);
      }
      return;
    }
    if (outcome.target == NavigationTarget::kRoom &&
      state_machine_.roomNavigationSucceeded() && state_machine_.roomVerified())
    {
      RCLCPP_INFO(
        get_logger(), "Room navigation verified at (%.3f, %.3f) after %zu attempt(s)",
        outcome.robot_pose.x, outcome.robot_pose.y, outcome.attempts);
      publishEvent(protocol::CompetitionEvent::kRoomReached);
      publishSearchRequest();
      return;
    }
    if (outcome.target == NavigationTarget::kDestination && state_machine_.destinationReached()) {
      RCLCPP_INFO(
        get_logger(), "Destination navigation verified at (%.3f, %.3f)",
        outcome.robot_pose.x, outcome.robot_pose.y);
      return;
    }
    RCLCPP_ERROR(get_logger(), "Navigation result did not match the current task state");
  }

void CoordinatorNode::publishEvent(protocol::CompetitionEvent event)
{
    const auto built = protocol::makeOutgoingMessage(event);
    if (!built.valid) {
      RCLCPP_ERROR(get_logger(), "Refused outgoing event: %s", built.reason.c_str());
      return;
    }
    publisher_->publish(built.message);
    RCLCPP_INFO(get_logger(), "Sent event: %s", built.message.message.c_str());
  }

void CoordinatorNode::handleEvent(const protocol::CompetitionMessage & message)
{
    using protocol::CompetitionEvent;
    switch (message.event) {
      case CompetitionEvent::kEnvironment:
        if (const auto * environment = environment_catalog_.find(message.detail)) {
          state_machine_.setEnvironment(environment->name);
          RCLCPP_INFO(get_logger(), "Loaded environment configuration: %s", environment->name.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Ignored unknown environment: %s", message.detail.c_str());
        }
        break;
      case CompetitionEvent::kAreYouReady:
        if (dispatchBlocked()) {
          RCLCPP_WARN(get_logger(),"Ready handshake blocked: cancellation not confirmed");
          break;
        }
        if (state_machine_.acceptReady()) {
          publishEvent(CompetitionEvent::kIAmReady);
        } else {
          RCLCPP_WARN(get_logger(), "Ignored Are_you_ready?: environment missing or state not ready");
        }
        break;
      case CompetitionEvent::kInstruction:
        if (state_machine_.acceptInstruction(message.detail)) {
          invalidateSearchRequest("new_instruction");
          if (parseCurrentInstruction(message.detail)) {
            if (simulate_modules_) {
              startSimulation();
            } else {
              startRoomNavigation();
            }
          }
        }
        break;
      case CompetitionEvent::kCorrectedInstruction:
        if (state_machine_.acceptInstruction(message.detail, true)) {
          invalidateSearchRequest("corrected_instruction");
          if (search_requests_enabled_ && navigation_executor_) stopNavigation();
          if (parseCurrentInstruction(message.detail)) {
            if (simulate_modules_) {
              startSimulation();
            } else {
              startRoomNavigation();
            }
          }
        }
        break;
      case CompetitionEvent::kTaskSucceeded:
        invalidateSearchRequest("task_succeeded");
        if (!state_machine_.moderatorSucceeded()) {
          RCLCPP_WARN(get_logger(), "Ignored Task_succeeded in the current state");
        }
        break;
      case CompetitionEvent::kTaskFailed:
        invalidateSearchRequest("task_failed");
        stopSimulation();
        if (navigation_executor_) {
          stopNavigation();
        }
        if (!state_machine_.moderatorFailed()) {
          RCLCPP_WARN(get_logger(), "Ignored Task_failed in the current state");
        }
        break;
      case CompetitionEvent::kMissionComplete:
        invalidateSearchRequest("mission_complete");
        stopSimulation();
        if (navigation_executor_) {
          stopNavigation();
        }
        state_machine_.missionCompleted();
        RCLCPP_INFO(get_logger(), "Mission complete received; shutting down safely");
        rclcpp::shutdown();
        break;
      default:
        RCLCPP_ERROR(
          get_logger(), "Incoming event reached an impossible dispatch path: %s",
          std::string(protocol::eventName(message.event)).c_str());
        break;
    }
  }

void CoordinatorNode::onMessage(const handyman_msgs::msg::HandymanMsg & ros_message)
{
    const auto parsed = protocol::parseIncomingMessage(ros_message);
    if (!parsed.valid) {
      RCLCPP_WARN(get_logger(), "Rejected competition message: %s", parsed.reason.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Received event: %s, detail: %s",
      std::string(protocol::eventName(parsed.message.event)).c_str(),
      parsed.message.detail.c_str());
    handleEvent(parsed.message);
  }

}  // namespace handyman_rebuild_ros2
