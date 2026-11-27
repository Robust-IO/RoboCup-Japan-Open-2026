#pragma once
#include "startup_clock_policy.hpp"
#include <map>
#include <mutex>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
namespace startup_clock {
struct Endpoint { int fd; int64_t gap_ms; };
inline std::map<long,Endpoint>& endpoints(){static std::map<long,Endpoint> value;return value;}
inline std::mutex& mutex(){static std::mutex value;return value;}
inline std::string message(int64_t gap){return "time_gap,"+std::to_string(gap/1000)+","+std::to_string(gap%1000);}
inline bool send_gap(int fd,int64_t gap){auto m=message(gap);return ::send(fd,m.data(),m.size(),MSG_NOSIGNAL|MSG_DONTWAIT)==static_cast<ssize_t>(m.size());}
inline bool initial(long key,int fd,int64_t gap){
 std::lock_guard<std::mutex> lock(mutex());
 if(!send_gap(fd,gap))return false;
 auto it=endpoints().find(key);if(it!=endpoints().end()){::close(it->second.fd);endpoints().erase(it);}
 int copy=::dup(fd);if(copy<0)return false;
 endpoints()[key]={copy,gap};return true;
}
inline void disconnected(long key){std::lock_guard<std::mutex> lock(mutex());auto it=endpoints().find(key);if(it!=endpoints().end()){::close(it->second.fd);endpoints().erase(it);}}
class Service : public rclcpp::Node {
 std::vector<Sample> samples_;
 rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
 static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 public:
 Service():Node("sigverse_startup_clock_service"){
  joints_=create_subscription<sensor_msgs::msg::JointState>("/hsrb/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::SharedPtr m){
   int64_t stamp=int64_t(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;
   double t=steady();
   if(!samples_.empty() && (stamp<=samples_.back().stamp_ns || t-samples_.back().received_s>.6))samples_.clear();
   samples_.push_back({t,(stamp-this->now().nanoseconds())/1e6,stamp,m->position});
   while(!samples_.empty() && t-samples_.front().received_s>3.5)samples_.erase(samples_.begin());
  });
  service_=create_service<std_srvs::srv::Trigger>("/sigverse/recalibrate_startup_clock",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
   for(const auto& topic:{"/hsrb/command_velocity","/hsrb/omni_base_controller/command","/hsrb/arm_trajectory_controller/command","/hsrb/gripper_controller/command"})
    if(count_publishers(topic)>0){r->success=false;r->message="motion_owner_present_stop_before_clock_calibration";return;}
   if(count_publishers("/hsrb/joint_states")!=1){r->success=false;r->message="one_joint_source_required";return;}
   auto d=assess(samples_,steady());r->success=d.usable;r->message=d.reason;
   if(!d.usable || !d.correction)return;
   std::lock_guard<std::mutex> lock(mutex());
   if(endpoints().size()!=1){r->success=false;r->message="exactly_one_native_unity_timesync_connection_required";return;}
   auto& e=endpoints().begin()->second;auto old=e.gap_ms;auto next=old+d.correction_ms;
   if(!send_gap(e.fd,next)){r->success=false;r->message="native_time_gap_send_failed";return;}
   e.gap_ms=next;samples_.clear();
   r->message="source_clock_adjustment_sent old_gap_ms="+std::to_string(old)+" new_gap_ms="+std::to_string(next)+" verify_fresh_feedback_before_motion";
   RCLCPP_INFO(get_logger(),"%s",r->message.c_str());
  });
 }
};
}
