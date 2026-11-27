#include "teleop_base.hpp"
#include <interactive_cleanup_msgs/msg/interactive_cleanup_msg.hpp>

namespace MsgIc 
{
  constexpr const char* IS_THIS_CORRECT = "Is_this_correct?";
  constexpr const char* POINT_IT_AGAIN  = "Point_it_again";
}

class InteractiveCleanupScenario : public TeleopScenarioInterface
{
public:
  InteractiveCleanupScenario() = default;

  void setup_ros(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
    pub_msg_ic_ = node_->create_publisher   <interactive_cleanup_msgs::msg::InteractiveCleanupMsg>("/interactive_cleanup/message/to_moderator", 10);
    sub_msg_ic_ = node_->create_subscription<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>("/interactive_cleanup/message/to_robot", 10, std::bind(&InteractiveCleanupScenario::recv_message, this, std::placeholders::_1));
  }

  void show_help(WINDOW* win_header, int start_row) override
  {
    if (!win_header) { return; }

    mvwprintw(win_header, start_row + 0, 2, " 1: Send Msg: %s", MsgCm::OBJECT_GRASPED);
    mvwprintw(win_header, start_row + 1, 2, " 2: Send Msg: %s", MsgCm::TASK_FINISHED);
    mvwprintw(win_header, start_row + 2, 2, " 6: Send Msg: %s", MsgIc::IS_THIS_CORRECT);
    mvwprintw(win_header, start_row + 3, 2, " 7: Send Msg: %s", MsgIc::POINT_IT_AGAIN);
    mvwprintw(win_header, start_row + 4, 2, " 9: Send Msg: %s", MsgCm::GIVE_UP);
  }

  bool handle_numeric_key(int key) override
  {
    if (!pub_msg_ic_) { return false; }

    switch (key)
    {
      case '1': send_message(MsgCm::OBJECT_GRASPED);  return true;
      case '2': send_message(MsgCm::TASK_FINISHED);   return true;
      case '6': send_message(MsgIc::IS_THIS_CORRECT); return true;
      case '7': send_message(MsgIc::POINT_IT_AGAIN);  return true;
      case '9': send_message(MsgCm::GIVE_UP);         return true;
      default:                                        return false;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher   <interactive_cleanup_msgs::msg::InteractiveCleanupMsg>::SharedPtr pub_msg_ic_;
  rclcpp::Subscription<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>::SharedPtr sub_msg_ic_;

  void send_message(const std::string& message)
  {
    RCLCPP_INFO(node_->get_logger(), "IC:Send msg:%s", message.c_str());
    interactive_cleanup_msgs::msg::InteractiveCleanupMsg interactive_cleanup_msg;
    interactive_cleanup_msg.message = message;
    pub_msg_ic_->publish(interactive_cleanup_msg);
  }

  void recv_message(const interactive_cleanup_msgs::msg::InteractiveCleanupMsg::SharedPtr message)
  {
    RCLCPP_INFO(node_->get_logger(),"IC:Subscribe:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message==MsgCm::ARE_YOU_READY)
    {
      interactive_cleanup_msgs::msg::InteractiveCleanupMsg i_am_ready_msg;
      i_am_ready_msg.message = MsgCm::I_AM_READY;
      pub_msg_ic_->publish(i_am_ready_msg);

      RCLCPP_INFO(node_->get_logger(), "IC:Send msg:%s", MsgCm::I_AM_READY);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto scenario = std::make_unique<InteractiveCleanupScenario>();
  CompetitionTeleopKey teleop_key(std::move(scenario), "teleop_interactive_cleanup");

  return teleop_key.run();
}
