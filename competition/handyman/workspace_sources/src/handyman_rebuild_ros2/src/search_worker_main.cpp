#include <cmath>
#include <thread>
#include "handyman_rebuild_ros2/search_binding_publisher.hpp"
#include "handyman_rebuild_ros2/search_map_evidence_client.hpp"

// One pinned search point per process. A navigation result is NOT an object
// search result or task completion. External supervisor owns process retirement.
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("handyman_search_worker");
  try {
    if (!node->declare_parameter<bool>("execution.enabled",false)) {
      RCLCPP_WARN(node->get_logger(),"Search execution disabled; no navigation clients created");
      rclcpp::shutdown();return 2;
    }
    auto param=[&](const char * name) {return node->declare_parameter<std::string>(name,"");};
    const auto config=param("catalog"),environment=param("environment"),room=param("room");
    const auto task=param("task_id"),point=param("point_id"),digest=param("map_sha256");
    const auto index=node->declare_parameter<int64_t>("point_index",-1);
    const auto lifetime=node->declare_parameter<double>("lifetime_sec",30.);
    if (index<0 || !std::isfinite(lifetime) || lifetime<3. || lifetime>600.)
      throw std::invalid_argument("invalid point index/lifetime");
    handyman_rebuild_ros2::EnvironmentCatalog catalog;std::string error;
    if (!catalog.loadFromFile(config,error)) throw std::invalid_argument(error);
    const auto env=catalog.find(environment);
    if (!env || !env->rooms.count(room) || static_cast<size_t>(index)>=env->rooms.at(room).search_points.size())
      throw std::invalid_argument("unknown environment/room/search point");
    handyman_rebuild_ros2::NavigationSettings settings;
    settings.action_name=node->declare_parameter<std::string>("navigation.action_name","navigate_to_pose");
    settings.search_behavior_tree=node->declare_parameter<std::string>("navigation.search_behavior_tree","");
    if (!settings.search_behavior_tree.empty()) settings.maximum_attempts=1;
    // In session-budget mode there is no separate 60 s goal timer cutting off
    // healthy navigation. The worker/session deadline and progress/lease guards
    // remain authoritative; phase-3 NavigationExecutor defaults are unchanged.
    if (node->declare_parameter<bool>("navigation.use_session_budget",false))
      settings.goal_timeout_sec=lifetime;
    handyman_rebuild_ros2::NavigationExecutor nav(node.get(),&catalog,settings);
    using Guard=handyman_rebuild_ros2::SearchExecutionGuard;
    handyman_rebuild_ros2::SearchMapEvidenceClient map(node.get(),task,point,digest,
      "/handyman/search/map_check/request","/handyman/search/map_check/reply");
    handyman_rebuild_ros2::SearchBindingPublisher binding(node.get(),task,point,digest,"/handyman/search/binding");
    Guard guard(node.get(),nav,task,{environment,room,point,static_cast<size_t>(index)},digest,
      {"/handyman/search/lease/request","/handyman/search/lease/reply",
       "/handyman/search/owned_goals","/handyman/search/owned_goal_ack",
       "/handyman/search/request","/handyman/search/execution_status"},[&]() {return map.evidence();});
    auto status=node->create_publisher<handyman_msgs::msg::HandymanMsg>("/handyman/search/execution_status",10);
    auto timeout_fault=[&]() {
      YAML::Node row;row["schema"]="handyman-search-worker-fault-v1";
      row["task_id"]=task;row["reason"]="worker_lifetime_exceeded";
      handyman_msgs::msg::HandymanMsg msg;msg.message="search_worker_fault";msg.detail=YAML::Dump(row);status->publish(msg);
    };
    using Clock=std::chrono::steady_clock;
    const auto deadline=Clock::now()+std::chrono::duration<double>(lifetime);
    bool dispatched=false,timed_out=false;
    bool retired=false;
    auto retire=node->create_subscription<handyman_msgs::msg::HandymanMsg>(
      "/handyman/search/retire",10,[&](const handyman_msgs::msg::HandymanMsg & msg) {
        if (timed_out || Clock::now()>=deadline || msg.message!="search_retire") return;
        try {
          const auto row=YAML::Load(msg.detail);
          if (row["schema"].as<std::string>()=="handyman-search-retire-v1" &&
            row["task_id"].as<std::string>()==task && guard.retirementReady(row["cancel_id"].as<std::string>()))
            retired=true;
        } catch (const YAML::Exception &) {return;}
      });
    auto stop_deadline=Clock::time_point::max();
    while(rclcpp::ok()) {
      rclcpp::spin_some(node);
      if (retired) {RCLCPP_INFO(node->get_logger(),"Confirmed search retirement");break;}
      if (Clock::now()>=deadline && !timed_out) {
        timed_out=true;nav.sealAndCancel();stop_deadline=Clock::now()+std::chrono::seconds(3);
      }
      if (timed_out) {
        timeout_fault();
        if (Clock::now()>=stop_deadline) break;
      } else if (!dispatched && guard.ready()) {
        dispatched=true;
        if (!guard.navigate([&](const auto & uuid,auto generation,const auto & candidate) {
              binding.accepted(uuid,generation,candidate);
            },[&](const auto & result) {
              RCLCPP_INFO(node->get_logger(),"Search navigation completed=%d reason=%s; retaining task ownership",
                result.success,result.reason.c_str());
              if (!result.success) {
                YAML::Node row;row["schema"]="handyman-search-worker-fault-v1";
                row["task_id"]=task;row["reason"]="search_navigation_failed";
                row["detail"]=result.reason;
                handyman_msgs::msg::HandymanMsg msg;
                msg.message="search_worker_fault";msg.detail=YAML::Dump(row);status->publish(msg);
              }
            })) throw std::runtime_error("guard rejected dispatch");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // Shutdown signal is not proof of drain; external supervision must cover it.
    nav.sealAndCancel();
    if (rclcpp::ok()) rclcpp::shutdown();
    return timed_out ? 4 : 0;
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(),"Search worker refused/stopped: %s",exc.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 3;
  }
}
