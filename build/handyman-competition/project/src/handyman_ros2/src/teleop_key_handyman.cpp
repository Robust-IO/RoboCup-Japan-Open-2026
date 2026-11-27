#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <handyman_msgs/msg/handyman_msg.hpp>

class HandymanTeleopKey
{
private:
  static const char KEYCODE_0 = 0x30;
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_6 = 0x36;
  static const char KEYCODE_9 = 0x39;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEYCODE_A = 0x61;
  static const char KEYCODE_B = 0x62;
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_D = 0x64;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_K = 0x6b;
  static const char KEYCODE_L = 0x6c;
  static const char KEYCODE_M = 0x6d;
  static const char KEYCODE_N = 0x6e;
  static const char KEYCODE_O = 0x6f;
  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;
  static const char KEYCODE_SPACE  = 0x20;

  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_ENVIRONMENT      = "Environment";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";

  const std::string MSG_I_AM_READY     = "I_am_ready";
  const std::string MSG_ROOM_REACHED   = "Room_reached";
  const std::string MSG_DOES_NOT_EXIST = "Does_not_exist";
  const std::string MSG_OBJECT_GRASPED = "Object_grasped";
  const std::string MSG_TASK_FINISHED  = "Task_finished";
  const std::string MSG_GIVE_UP        = "Give_up";

public:
  HandymanTeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void messageCallback(const handyman_msgs::msg::HandymanMsg::ConstSharedPtr message);
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
  void sendMessage(const std::string &message);
  void moveBaseTwist(double linear_x, double linear_y, double angular_z);
  void moveBaseJointTrajectory(double linear_x, double linear_y, double theta, double duration_sec);
  void operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec);
  void operateArm(const std::string &name, const double position, const double duration_sec);
  void operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos);
  double getDurationRot(const double next_pos, const double current_pos);
  void operateHand(bool grasp);

  void showHelp();
  int run(int argc, char **argv);

private:
  bool is_received_are_you_ready_;
  bool is_received_environment_;

  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
  double arm_flex_joint_pos_;
  double wrist_flex_joint_pos_;

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr sub_msg_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr    pub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   sub_joint_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_base_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};


HandymanTeleopKey::HandymanTeleopKey()
{
  is_received_are_you_ready_ = false;
  is_received_environment_   = false;

  arm_lift_joint_pos1_   = 0.0;
  arm_lift_joint_pos2_   = 0.0;
  arm_flex_joint_pos_    = 0.0;
  wrist_flex_joint_pos_  = 0.0;
}


void HandymanTeleopKey::rosSigintHandler(int sig)
{
  (void)sig;
  rclcpp::shutdown();
}


int HandymanTeleopKey::canReceive( int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  ret = select( fd+1 , &fdset , NULL , NULL , &timeout );
  return ret;
}

void HandymanTeleopKey::messageCallback(const handyman_msgs::msg::HandymanMsg::ConstSharedPtr message)
{
  if(message->message.c_str()==MSG_ARE_YOU_READY && is_received_are_you_ready_){ return; }
  if(message->message.c_str()==MSG_ENVIRONMENT   && is_received_environment_)  { return; }

  RCLCPP_INFO(node_->get_logger(), "Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

  if(message->message.c_str()==MSG_ARE_YOU_READY){ is_received_are_you_ready_ = true; }
  if(message->message.c_str()==MSG_ENVIRONMENT  ){ is_received_environment_   = true; }

  if(message->message.c_str()==MSG_TASK_SUCCEEDED || message->message.c_str()==MSG_TASK_FAILED || message->message.c_str()==MSG_MISSION_COMPLETE)
  {
    is_received_are_you_ready_ = false;
    is_received_environment_   = false;
  }
}

void HandymanTeleopKey::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
{
  for(size_t i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i] == "arm_lift_joint")
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "arm_flex_joint")
    {
      arm_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "wrist_flex_joint")
    {
      wrist_flex_joint_pos_ = joint_state->position[i];
    }
  }
}

void HandymanTeleopKey::sendMessage(const std::string &message)
{
  RCLCPP_INFO(node_->get_logger(), "Send message:%s", message.c_str());

  handyman_msgs::msg::HandymanMsg handyman_msg;
  handyman_msg.message = message;
  pub_msg_->publish(handyman_msg);
}

void HandymanTeleopKey::moveBaseTwist(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void HandymanTeleopKey::moveBaseJointTrajectory(double linear_x, double linear_y, double theta, double duration_sec)
{
  if(tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero) == false)
  {
    return;
  }

  geometry_msgs::msg::PointStamped basefootprint_2_target;
  geometry_msgs::msg::PointStamped odom_2_target;
  basefootprint_2_target.header.frame_id = "base_footprint";
  basefootprint_2_target.header.stamp = rclcpp::Time(0);
  basefootprint_2_target.point.x = linear_x;
  basefootprint_2_target.point.y = linear_y;
  tf_buffer_->transform(basefootprint_2_target, odom_2_target, "odom");

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  tf2::Quaternion currentRotation;
  tf2::fromMsg(transform_stamped.transform.rotation, currentRotation);
  tf2::Matrix3x3 mat(currentRotation);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("odom_x");
  joint_trajectory.joint_names.push_back("odom_y");
  joint_trajectory.joint_names.push_back("odom_t");

  trajectory_msgs::msg::JointTrajectoryPoint omni_joint_point;
  omni_joint_point.positions = {odom_2_target.point.x, odom_2_target.point.y, yaw + theta};
  omni_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  joint_trajectory.points.push_back(omni_joint_point);
  pub_base_trajectory_->publish(joint_trajectory);
}


void HandymanTeleopKey::operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("arm_lift_joint");
  joint_trajectory.joint_names.push_back("arm_flex_joint");
  joint_trajectory.joint_names.push_back("arm_roll_joint");
  joint_trajectory.joint_names.push_back("wrist_flex_joint");
  joint_trajectory.joint_names.push_back("wrist_roll_joint");

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, 0.0f, wrist_flex_pos, 0.0f};

  arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  pub_arm_trajectory_->publish(joint_trajectory);
}

void HandymanTeleopKey::operateArm(const std::string &name, const double position, const double duration_sec)
{
  if(name == "arm_lift_joint")
  {
    this->operateArm(position, arm_flex_joint_pos_, wrist_flex_joint_pos_, duration_sec);
  }
  else if(name == "arm_flex_joint")
  {
    this->operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, position, wrist_flex_joint_pos_, duration_sec);
  }
  else if(name == "wrist_flex_joint")
  {
    this->operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, arm_flex_joint_pos_, position, duration_sec);
  }
}

void HandymanTeleopKey::operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos)
{
  double duration = std::max(this->getDurationRot(arm_flex_pos, arm_flex_joint_pos_), this->getDurationRot(wrist_flex_pos, wrist_flex_joint_pos_));

  this->operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, arm_flex_pos, wrist_flex_pos, duration);
}

double HandymanTeleopKey::getDurationRot(const double next_pos, const double current_pos)
{
  return std::max<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
}

void HandymanTeleopKey::operateHand(bool is_hand_open)
{
  std::vector<std::string> joint_names {"hand_motor_joint"};
  std::vector<double> positions;

  if(is_hand_open)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Grasp");
    positions.push_back(-0.105);
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "Open hand");
    positions.push_back(+1.239);
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(2);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);
  pub_gripper_trajectory_->publish(joint_trajectory);
}


void HandymanTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("space      : Stop HSR");
  puts("---------------------------");
  puts("Move HSR Linearly (1m)");
  puts("  u   i   o  ");
  puts("  j   k   l  ");
  puts("  m   ,   .  ");
  puts("---------------------------");
  puts("q/z : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("y : Up   Torso");
  puts("h : Stop Torso");
  puts("n : Down Torso");
  puts("---------------------------");
  puts("a : Rotate Arm - Vertical");
  puts("b : Rotate Arm - Upward");
  puts("c : Rotate Arm - Horizontal");
  puts("d : Rotate Arm - Downward");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("---------------------------");
  puts(("0 : Send "+MSG_I_AM_READY).c_str());
  puts(("1 : Send "+MSG_ROOM_REACHED).c_str());
  puts(("2 : Send "+MSG_OBJECT_GRASPED).c_str());
  puts(("3 : Send "+MSG_TASK_FINISHED).c_str());
  puts(("6 : Send "+MSG_DOES_NOT_EXIST).c_str());
  puts(("9 : Send "+MSG_GIVE_UP).c_str());
}

int HandymanTeleopKey::run(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  node_ = std::make_shared<rclcpp::Node>("handyman_teleop_key");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  char c;

  /////////////////////////////////////////////
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  showHelp();

  signal(SIGINT, rosSigintHandler);

  rclcpp::Rate loop_rate(40);

  sub_msg_                = node_->create_subscription<handyman_msgs::msg::HandymanMsg>("/handyman/message/to_robot", 100, std::bind(&HandymanTeleopKey::messageCallback, this, std::placeholders::_1));
  pub_msg_                = node_->create_publisher<handyman_msgs::msg::HandymanMsg>("/handyman/message/to_moderator", 10);

  sub_joint_state_        = node_->create_subscription<sensor_msgs::msg::JointState>("/hsrb/joint_states", 10, std::bind(&HandymanTeleopKey::jointStateCallback, this, std::placeholders::_1));
  pub_base_twist_         = node_->create_publisher<geometry_msgs::msg::Twist>("/hsrb/command_velocity", 10);
  pub_base_trajectory_    = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/omni_base_controller/command", 10);
  pub_arm_trajectory_     = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  pub_gripper_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/gripper_controller/command", 10);


  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

  float move_speed = 1.0f;
  bool is_hand_open = false;

  std::string arm_lift_joint_name   = "arm_lift_joint";
  std::string arm_flex_joint_name   = "arm_flex_joint";
  std::string wrist_flex_joint_name = "wrist_flex_joint";

  while (rclcpp::ok())
  {
    if(canReceive(kfd))
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      switch(c)
      {
        case KEYCODE_0:
        {
          sendMessage(MSG_I_AM_READY);
          break;
        }
        case KEYCODE_1:
        {
          sendMessage(MSG_ROOM_REACHED);
          break;
        }
        case KEYCODE_2:
        {
          sendMessage(MSG_OBJECT_GRASPED);
          break;
        }
        case KEYCODE_3:
        {
          sendMessage(MSG_TASK_FINISHED);
          break;
        }
        case KEYCODE_6:
        {
          sendMessage(MSG_DOES_NOT_EXIST);
          break;
        }
        case KEYCODE_9:
        {
          sendMessage(MSG_GIVE_UP);
          break;
        }
        case KEYCODE_UP:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Go Forward");
          moveBaseTwist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Go Backward");
          moveBaseTwist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Go Right");
          moveBaseTwist(0.0, 0.0, -angular_coef*move_speed);
          break;
        }
        case KEYCODE_LEFT:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Go Left");
          moveBaseTwist(0.0, 0.0, +angular_coef*move_speed);
          break;
        }
        case KEYCODE_SPACE:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Stop");
          moveBaseTwist(0.0, 0.0, 0.0);
          break;
        }
        case KEYCODE_U:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Left Forward");
          moveBaseJointTrajectory(+1.0, +1.0, +M_PI_4, 10);
          break;
        }
        case KEYCODE_I:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Forward");
          moveBaseJointTrajectory(+1.0, 0.0, 0.0, 10);
          break;
        }
        case KEYCODE_O:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Right Forward");
          moveBaseJointTrajectory(+1.0, -1.0, -M_PI_4, 10);
          break;
        }
        case KEYCODE_J:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Left");
          moveBaseJointTrajectory(0.0, +1.0, +M_PI_2, 10);
          break;
        }
        case KEYCODE_K:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Stop");
          moveBaseJointTrajectory(0.0, 0.0, 0.0, 0.5);
          break;
        }
        case KEYCODE_L:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Right");
          moveBaseJointTrajectory(0.0, -1.0, -M_PI_2, 10);
          break;
        }
        case KEYCODE_M:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Left Backward");
          moveBaseJointTrajectory(-1.0, +1.0, +M_PI_2+M_PI_4, 10);
          break;
        }
        case KEYCODE_COMMA:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Backward");
          moveBaseJointTrajectory(-1.0, 0.0, +M_PI, 10);
          break;
        }
        case KEYCODE_PERIOD:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Right Backward");
          moveBaseJointTrajectory(-1.0, -1.0, -M_PI_2-M_PI_4, 10);
          break;
        }
        case KEYCODE_Q:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case KEYCODE_Z:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case KEYCODE_Y:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Up Torso");
          operateArm(arm_lift_joint_name, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_H:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Stop Torso");
          operateArm(arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        }
        case KEYCODE_N:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Down Torso");
          operateArm(arm_lift_joint_name, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_A:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Rotate Arm - Vertical");
          operateArmFlex(0.0, -1.57);
          break;
        }
        case KEYCODE_B:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Rotate Arm - Upward");
          operateArmFlex(-0.785, -0.785);
          break;
        }
        case KEYCODE_C:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Rotate Arm - Horizontal");
          operateArmFlex(-1.57, 0.0);
          break;
        }
        case KEYCODE_D:
        {
          RCLCPP_DEBUG(node_->get_logger(), "Rotate Arm - Downward");
          operateArmFlex(-2.2, 0.35);
          break;
        }
        case KEYCODE_G:
        {
          operateHand(is_hand_open);

          is_hand_open = !is_hand_open;
          break;
        }
      }
    }

    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  HandymanTeleopKey handyman_teleop_key;
  int ret = handyman_teleop_key.run(argc, argv);

  rclcpp::shutdown();
  return ret;
}
