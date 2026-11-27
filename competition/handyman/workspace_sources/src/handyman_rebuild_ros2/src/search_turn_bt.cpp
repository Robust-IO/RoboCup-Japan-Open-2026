#include <cmath>
#include <memory>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "handyman_rebuild_ros2/search_turn_profile.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace handyman_rebuild_ros2 {
class SearchSpinAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin> {
public:
  SearchSpinAction(const std::string & name,const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::Spin>(name,"search_spin",conf) {}
  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<double>("angle")});
  }
  void on_tick() override {
    double angle;
    if (!getInput("angle",angle) || !std::isfinite(angle) || std::abs(angle)>1.35)
      throw BT::RuntimeError("Invalid search spin angle");
    goal_.target_yaw=angle;
    // The original session/worker lease remains the deadline authority.
    goal_.time_allowance=rclcpp::Duration::from_seconds(0.);
  }
};
class PrepareSearchTurn : public BT::SyncActionNode {
public:
  PrepareSearchTurn(const std::string & name,const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name,config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"), BT::OutputPort<double>("angle")};
  }
  BT::NodeStatus tick() override {
    try {
      const auto node=config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      const auto tf=config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
      geometry_msgs::msg::PoseStamped goal;
      if (!getInput("goal",goal) || goal.header.frame_id!="map") return BT::NodeStatus::FAILURE;
      const auto pose=tf->lookupTransform("map","base_footprint",tf2::TimePointZero);
      const double age=(node->now()-rclcpp::Time(pose.header.stamp)).seconds();
      const double distance=std::hypot(goal.pose.position.x-pose.transform.translation.x,
                                      goal.pose.position.y-pose.transform.translation.y);
      const double angle=turnError(tf2::getYaw(goal.pose.orientation),tf2::getYaw(pose.transform.rotation));
      // Leave a margin under the independent observer's 15 cm arrival gate.
      if (!std::isfinite(distance) || !std::isfinite(angle) || age<-.02 || age>.5 ||
          distance>.14 || std::abs(angle)>1.35) {
        RCLCPP_ERROR(node->get_logger(),"Search turn refused: distance=%.3f angle=%.3f TF age=%.3f",distance,angle,age);
        return BT::NodeStatus::FAILURE;
      }
      setOutput("angle",angle);
      RCLCPP_INFO(node->get_logger(),"Search turn-only: distance=%.3f angle=%.3f",distance,angle);
      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception &) {return BT::NodeStatus::FAILURE;}
  }
};
}
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<handyman_rebuild_ros2::PrepareSearchTurn>("PrepareSearchTurn");
  factory.registerNodeType<handyman_rebuild_ros2::SearchSpinAction>("SearchSpinAction");
}
