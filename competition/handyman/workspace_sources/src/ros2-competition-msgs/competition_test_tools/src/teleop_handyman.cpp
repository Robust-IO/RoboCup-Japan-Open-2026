#include "teleop_base.hpp"
#include <handyman_msgs/msg/handyman_msg.hpp>

namespace MsgHm 
{
  constexpr const char* ENVIRONMENT    = "Environment";
  constexpr const char* ROOM_REACHED   = "Room_reached";
  constexpr const char* DOES_NOT_EXIST = "Does_not_exist";
}

class HandymanScenario : public TeleopScenarioInterface
{
public:
  HandymanScenario() = default;
  
  void setup_ros(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
    pub_msg_hm_ = node_->create_publisher   <handyman_msgs::msg::HandymanMsg>("/handyman/message/to_moderator", 10);
    sub_msg_hm_ = node_->create_subscription<handyman_msgs::msg::HandymanMsg>("/handyman/message/to_robot", 10, std::bind(&HandymanScenario::recv_message, this, std::placeholders::_1));
  }

  void show_help(WINDOW* win_header, int start_row) override
  {
    if (!win_header) { return; }

    mvwprintw(win_header, start_row + 0, 2, " 1: Send Msg: %s", MsgHm::ROOM_REACHED);
    mvwprintw(win_header, start_row + 1, 2, " 2: Send Msg: %s", MsgCm::OBJECT_GRASPED);
    mvwprintw(win_header, start_row + 2, 2, " 3: Send Msg: %s", MsgCm::TASK_FINISHED);
    mvwprintw(win_header, start_row + 3, 2, " 6: Send Msg: %s", MsgHm::DOES_NOT_EXIST);
    mvwprintw(win_header, start_row + 4, 2, " 9: Send Msg: %s", MsgCm::GIVE_UP);
  }

  bool handle_numeric_key(int key) override
  {
    if (!pub_msg_hm_) { return false; }

    switch (key)
    {
      case '1': send_message(MsgHm::ROOM_REACHED);   return true;
      case '2': send_message(MsgCm::OBJECT_GRASPED); return true;
      case '3': send_message(MsgCm::TASK_FINISHED);  return true;
      case '6': send_message(MsgHm::DOES_NOT_EXIST); return true;
      case '9': send_message(MsgCm::GIVE_UP);        return true;
      default:                                       return false;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher   <handyman_msgs::msg::HandymanMsg>::SharedPtr pub_msg_hm_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr sub_msg_hm_;

  void send_message(const std::string& message)
  {
    RCLCPP_INFO(node_->get_logger(), "HM:Send msg:%s", message.c_str());
    handyman_msgs::msg::HandymanMsg handyman_msg;
    handyman_msg.message = message;
    pub_msg_hm_->publish(handyman_msg);
  }

  void recv_message(const handyman_msgs::msg::HandymanMsg::SharedPtr message)
  {
    RCLCPP_INFO(node_->get_logger(),"HM:Subscribe:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message==MsgCm::ARE_YOU_READY)
    {
      handyman_msgs::msg::HandymanMsg i_am_ready_msg;
      i_am_ready_msg.message = MsgCm::I_AM_READY;
      pub_msg_hm_->publish(i_am_ready_msg);

      RCLCPP_INFO(node_->get_logger(), "HM:Send msg:%s", MsgCm::I_AM_READY);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto scenario = std::make_unique<HandymanScenario>();
  CompetitionTeleopKey teleop_key(std::move(scenario), "teleop_handyman");

  return teleop_key.run();
}
