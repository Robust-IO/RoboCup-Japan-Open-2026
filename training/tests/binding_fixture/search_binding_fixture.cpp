// Isolated test only: invokes the real NavigationExecutor against a fake Nav2 server.
#include <cstdlib>
#include <thread>
#include "handyman_rebuild_ros2/search_binding_publisher.hpp"
#include "handyman_rebuild_ros2/search_cancel_reporter.hpp"
#include "handyman_rebuild_ros2/search_supervisor_lease.hpp"
#include "handyman_rebuild_ros2/search_goal_registration.hpp"
#include "handyman_rebuild_ros2/search_execution_guard.hpp"
#include "handyman_rebuild_ros2/search_map_evidence_client.hpp"
int main(int argc,char ** argv) {
  if (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="73" ||
      !std::getenv("ROS_LOCALHOST_ONLY") || std::string(std::getenv("ROS_LOCALHOST_ONLY"))!="1" || argc!=6) return 2;
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("search_binding_fixture");
  handyman_rebuild_ros2::EnvironmentCatalog catalog; std::string error;
  if (!catalog.loadFromFile(argv[1],error)) return 3;
  handyman_rebuild_ros2::NavigationSettings settings;
  settings.maximum_attempts=2; settings.goal_timeout_sec=10.;
  settings.action_name="/handyman_test/navigate_to_pose";
  handyman_rebuild_ros2::NavigationExecutor nav(node.get(),&catalog,settings);
  handyman_rebuild_ros2::SearchBindingPublisher publisher(node.get(),argv[3],argv[4],argv[5],"/handyman_test/search_binding");
  const bool registration_ack_enabled=std::getenv("HANDYMAN_TEST_REG_ACK")!=nullptr;
  const bool unified=std::getenv("HANDYMAN_TEST_UNIFIED_GUARD")!=nullptr;
  std::unique_ptr<handyman_rebuild_ros2::SearchMapEvidenceClient> map_client;
  if (std::getenv("HANDYMAN_TEST_LIVE_MAP"))
    map_client=std::make_unique<handyman_rebuild_ros2::SearchMapEvidenceClient>(node.get(),argv[3],argv[4],argv[5],
      "/handyman_test/map_check_request","/handyman_test/map_check_reply");
  std::unique_ptr<handyman_rebuild_ros2::SearchExecutionGuard> guard;
  using Guard=handyman_rebuild_ros2::SearchExecutionGuard;
  auto map_state=std::getenv("HANDYMAN_TEST_MAP_PENDING") ? Guard::MapState::pending : Guard::MapState::verified;
  auto map_control=node->create_subscription<handyman_msgs::msg::HandymanMsg>(
    "/handyman_test/map_control",10,[&](const handyman_msgs::msg::HandymanMsg & msg) {
      if (msg.detail==argv[3] && msg.message=="revoke") map_state=Guard::MapState::revoked;
    });
  if (unified) guard=std::make_unique<handyman_rebuild_ros2::SearchExecutionGuard>(node.get(),nav,
    argv[3],Guard::Target{argv[2],"kitchen",argv[4],0},argv[5],handyman_rebuild_ros2::SearchExecutionGuard::Topics{
      "/handyman_test/lease_request","/handyman_test/lease_reply",
      registration_ack_enabled ? "/handyman_test/owned_goals_raw" : "/handyman_test/owned_goals",
      registration_ack_enabled ? "/handyman_test/owned_goal_ack_raw" : "/handyman_test/owned_goal_ack",
      "/handyman_test/owned_request","/handyman_test/owned_status"},[&]() {
        return map_client ? map_client->evidence() : Guard::MapEvidence{map_state,argv[3],argv[4],argv[5]};
      });
  std::unique_ptr<handyman_rebuild_ros2::SearchSupervisorLease> lease;
  if (!unified && std::getenv("HANDYMAN_TEST_SUPERVISOR_LEASE")) {
    lease=std::make_unique<handyman_rebuild_ros2::SearchSupervisorLease>(node.get(),nav,argv[3],
      handyman_rebuild_ros2::SearchSupervisorLease::Topics{
        "/handyman_test/lease_request","/handyman_test/lease_reply","/handyman_test/owned_status"});
  }
  std::unique_ptr<handyman_rebuild_ros2::SearchGoalRegistration> registration;
  if (!unified) registration=std::make_unique<handyman_rebuild_ros2::SearchGoalRegistration>(node.get(),nav,argv[3],argv[4],argv[5],
    handyman_rebuild_ros2::SearchGoalRegistration::Topics{
      registration_ack_enabled ? "/handyman_test/owned_goals_raw" : "/handyman_test/owned_goals",
      "/handyman_test/owned_goal_ack_raw","/handyman_test/owned_status"},registration_ack_enabled);
  bool started=false, done=false;
  bool cancelling=false;
  auto cancel_ack=node->create_publisher<handyman_msgs::msg::HandymanMsg>(
    "/handyman_test/search_cancel_ack",10);
  nav.setCancellationObserver([&](const auto & uuid,const std::string & state) {
    std::ostringstream key;
    for (auto byte:uuid) key << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    handyman_msgs::msg::HandymanMsg msg;
    msg.message=state;msg.detail=key.str();cancel_ack->publish(msg);
  });
  auto cancel_deadline=std::chrono::steady_clock::time_point::max();
  std::unique_ptr<handyman_rebuild_ros2::SearchCancelReporter> reporter;
  if (!unified && std::getenv("HANDYMAN_TEST_WORKER_STATUS")) {
    reporter = std::make_unique<handyman_rebuild_ros2::SearchCancelReporter>(node.get(), nav,
      argv[3], "/handyman_test/search_request", "/handyman_test/search_execution_status",[&]() {
        if (!registration->healthy() || (lease && lease->failed())) return std::string("cancel_failed");
        return std::string(registration->settled() ? "cancel_drained" : "cancel_received");
      });
  }
  auto cancel_sub=node->create_subscription<handyman_msgs::msg::HandymanMsg>(
    "/handyman_test/search_cancel",10,[&](const handyman_msgs::msg::HandymanMsg & msg) {
      if (msg.message!="cancel_search" || msg.detail!=argv[3] || cancelling) return;
      cancelling=true;started=true;nav.cancel();
      handyman_msgs::msg::HandymanMsg ack;
      ack.message="executor_cancel_called";ack.detail=argv[3];cancel_ack->publish(ack);
      // Keep spinning so the directed cancellation (including a late accepted
      // handle) can be delivered. The test separately checks server acknowledgement.
      cancel_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    });
  auto timer=node->create_wall_timer(std::chrono::milliseconds(1000),[&]() {
    if (lease && !lease->dispatchAllowed()) return;
    if (registration && !registration->healthy()) return;
    if (guard && !guard->ready()) return;
    if (started || (reporter && reporter->cancelling())) return;
    started=true;
    if (guard) {
      if (!guard->navigate(
        [&](const auto & uuid,auto token,const auto & candidate){publisher.accepted(uuid,token,candidate);},
        [&](const auto & outcome){done=true;RCLCPP_INFO(node->get_logger(),"completed=%d",outcome.success);})) done=true;
      return;
    }
    if (!nav.navigateToSearchPoint(argv[2],"kitchen",0,
      [&](const auto & uuid,auto token,const auto & candidate){publisher.accepted(uuid,token,candidate);},
      [&](const auto & outcome){done=true; RCLCPP_INFO(node->get_logger(),"completed=%d",outcome.success);})) done=true;
  });
  auto end=std::chrono::steady_clock::now()+std::chrono::seconds(20);
  // A finished navigation is not the end of task ownership. In reporter mode,
  // remain available for cancellation while the vision observer finishes.
  // The 20-second outer limit is a test watchdog, not production lifecycle policy.
  while(rclcpp::ok() && (!done || reporter || guard) && std::chrono::steady_clock::now()<end &&
    std::chrono::steady_clock::now()<cancel_deadline) {
    if (((reporter && reporter->cancelling()) || (guard && guard->cancelling())) && !cancelling) {
      cancelling=true; started=true;
      cancel_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(2500);
      handyman_msgs::msg::HandymanMsg ack;
      ack.message="executor_cancel_called";ack.detail=argv[3];cancel_ack->publish(ack);
    }
    rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  nav.cancel(); rclcpp::shutdown(); return (done||cancelling)?0:4;
}
