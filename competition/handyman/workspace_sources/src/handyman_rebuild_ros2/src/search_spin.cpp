#include <chrono>
#include <cmath>
#include "nav2_behaviors/plugins/spin.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "handyman_rebuild_ros2/search_turn_profile.hpp"

namespace handyman_rebuild_ros2 {
// Keep Nav2's action lifecycle, exact-goal cancellation and collision checker.
// Only search_spin uses this implementation; normal navigation is untouched.
class SearchSpin : public nav2_behaviors::Spin {
  using Clock=std::chrono::steady_clock;
  Clock::time_point last_;
  double speed_{0}, start_x_{0}, start_y_{0}, start_yaw_{0};
  bool pose(geometry_msgs::msg::PoseStamped & p) {
    if (!nav2_util::getCurrentPose(p,*tf_,global_frame_,robot_base_frame_,.1)) return false;
    const double age=(clock_->now()-rclcpp::Time(p.header.stamp)).seconds();
    return age>=-.02 && age<=.5 && std::isfinite(p.pose.position.x) &&
      std::isfinite(p.pose.position.y) && std::isfinite(tf2::getYaw(p.pose.orientation));
  }
  nav2_behaviors::Status fail(const char * reason) {
    stopRobot();RCLCPP_ERROR(logger_,"Search turn stopped: %s",reason);
    return nav2_behaviors::Status::FAILED;
  }
public:
  nav2_behaviors::Status onRun(const std::shared_ptr<const nav2_msgs::action::Spin::Goal> goal) override {
    geometry_msgs::msg::PoseStamped p;
    if (!std::isfinite(goal->target_yaw) || std::abs(goal->target_yaw)>1.35 || !pose(p))
      return fail("invalid angle or stale TF");
    start_x_=p.pose.position.x;start_y_=p.pose.position.y;start_yaw_=tf2::getYaw(p.pose.orientation);
    speed_=0.;last_=Clock::now();
    return Spin::onRun(goal);
  }
  nav2_behaviors::Status onCycleUpdate() override {
    geometry_msgs::msg::PoseStamped p;
    const auto now=Clock::now();
    const double dt=std::chrono::duration<double>(now-last_).count();last_=now;
    if (!pose(p)) return fail("stale or missing TF");
    if (std::hypot(p.pose.position.x-start_x_,p.pose.position.y-start_y_)>.03)
      return fail("translation drift during turn");
    const double yaw=tf2::getYaw(p.pose.orientation);
    const double traveled=turnError(yaw,start_yaw_);
    const double direction=cmd_yaw_<0 ? -1. : 1.;
    const double remaining=std::abs(cmd_yaw_)-direction*traveled;
    feedback_->angular_distance_traveled=static_cast<float>(traveled);
    action_server_->publish_feedback(feedback_);
    if (dt>.5 || remaining<-.10 || direction*traveled<-.10)
      return fail("feedback gap or excessive angular deviation");
    if (remaining<=.035) {stopRobot();return nav2_behaviors::Status::SUCCEEDED;}
    if (command_time_allowance_.seconds()>0 && clock_->now()>end_time_)
      return fail("action deadline");
    try {speed_=turnSpeed(remaining,speed_,dt);} catch (...) {return fail("invalid turn profile");}
    geometry_msgs::msg::Twist cmd;cmd.angular.z=direction*speed_;
    // Check the entire remaining swept footprint, including the current pose,
    // not only the already-traveled angle. Fail closed on unknown/invalid data.
    geometry_msgs::msg::Pose2D probe;probe.x=p.pose.position.x;probe.y=p.pose.position.y;
    const int steps=std::max(1,static_cast<int>(std::ceil((remaining+.05)/.025)));
    try {
      for (int i=0;i<=steps;++i) {
        probe.theta=yaw+direction*(remaining+.05)*i/steps;
        if (!collision_checker_->isCollisionFree(probe,i==0)) return fail("swept footprint blocked");
      }
    } catch (...) {return fail("collision check unavailable");}
    vel_pub_->publish(cmd);
    return nav2_behaviors::Status::RUNNING;
  }
  void onActionCompletion() override {stopRobot();}
};
}
PLUGINLIB_EXPORT_CLASS(handyman_rebuild_ros2::SearchSpin,nav2_core::Behavior)
