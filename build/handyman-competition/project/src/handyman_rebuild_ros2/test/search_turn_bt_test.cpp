#include <cassert>
#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

int main(int argc,char ** argv) {
  assert(argc==2);
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("search_turn_bt_test");
  BT::BehaviorTreeFactory factory;factory.registerFromPlugin(argv[1]);
  auto run=[&](double dx,double yaw,double age,bool available) {
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto bb=BT::Blackboard::create();bb->set("node",node);bb->set("tf_buffer",tf);
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id="map";t.child_frame_id="base_footprint";
    t.header.stamp=node->now()-rclcpp::Duration::from_seconds(age);
    tf2::Quaternion q;q.setRPY(0.,0.,3.1);t.transform.rotation=tf2::toMsg(q);
    if (available) tf->setTransform(t,"test",false);
    geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";
    goal.pose.position.x=dx;q.setRPY(0.,0.,yaw);goal.pose.orientation=tf2::toMsg(q);bb->set("goal",goal);
    auto tree=factory.createTreeFromText("<root main_tree_to_execute='Test'><BehaviorTree ID='Test'><PrepareSearchTurn goal='{goal}' angle='{angle}'/></BehaviorTree></root>",bb);
    return tree.tickRoot();
  };
  assert(run(.104,3.1+.785398,0.,true)==BT::NodeStatus::SUCCESS);
  assert(run(.141,3.5,0.,true)==BT::NodeStatus::FAILURE);
  assert(run(.1,3.5,1.,true)==BT::NodeStatus::FAILURE);
  assert(run(.1,3.5,0.,false)==BT::NodeStatus::FAILURE);
  assert(run(.1,0.,0.,true)==BT::NodeStatus::FAILURE);
  rclcpp::shutdown();
}
