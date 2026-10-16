#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace handyman_rebuild_ros2
{

class InitialPoseLocalizer : public rclcpp::Node
{
public:
  InitialPoseLocalizer()
  : Node("initial_pose_localizer"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    broadcaster_(this)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    initial_x_ = declare_parameter<double>("initial_pose.x", 0.0);
    initial_y_ = declare_parameter<double>("initial_pose.y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_pose.yaw", 0.0);
    timer_ = create_wall_timer(100ms, [this]() {alignFrames();});
    RCLCPP_INFO(
      get_logger(), "Waiting for %s -> %s before aligning it with %s",
      odom_frame_.c_str(), base_frame_.c_str(), map_frame_.c_str());
  }

private:
  void alignFrames()
  {
    geometry_msgs::msg::TransformStamped odom_to_base_message;
    try {
      odom_to_base_message = tf_buffer_.lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }

    tf2::Transform odom_to_base;
    tf2::fromMsg(odom_to_base_message.transform, odom_to_base);
    tf2::Quaternion initial_rotation;
    initial_rotation.setRPY(0.0, 0.0, initial_yaw_);
    tf2::Transform map_to_initial_base(initial_rotation, tf2::Vector3(initial_x_, initial_y_, 0.0));
    const tf2::Transform map_to_odom = map_to_initial_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = tf2::toMsg(map_to_odom);
    broadcaster_.sendTransform(message);
    timer_->cancel();
    RCLCPP_INFO(
      get_logger(),
      "Aligned %s to initial map pose (%.3f, %.3f, %.3f) using the first Unity odometry pose",
      odom_frame_.c_str(), initial_x_, initial_y_, initial_yaw_);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace handyman_rebuild_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handyman_rebuild_ros2::InitialPoseLocalizer>());
  rclcpp::shutdown();
  return 0;
}
