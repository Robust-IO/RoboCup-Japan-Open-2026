#include <cstdio>
#include <csignal>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

// 原 ROS1: <ros/ros.h> -> ROS2: rclcpp
#include <rclcpp/rclcpp.hpp>  // 原 ROS1: ros::NodeHandle / ros::Rate / ros::Time 等

#include <geometry_msgs/msg/twist.hpp>                // 原 ROS1: geometry_msgs/Twist.h
#include <sensor_msgs/msg/joint_state.hpp>           // 原 ROS1: sensor_msgs/JointState.h
#include <trajectory_msgs/msg/joint_trajectory.hpp>  // 原 ROS1: trajectory_msgs/JointTrajectory.h
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// 原 ROS1: tf/transform_listener.h -> ROS2: tf2_ros + tf2_geometry_msgs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>  // 原 ROS1: tf::Matrix3x3 + getRPY -> ROS2: tf2::getYaw 辅助函数

// 原 ROS1: <human_navigation/...> -> ROS2: human_navigation_msgs/msg/...
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>

class HSRKeyTeleopNode : public rclcpp::Node
{
public:
  HSRKeyTeleopNode()
  : Node("hsr_teleop_key")
  {
    // 原 ROS1: 使用 NodeHandle 参数，ROS2: 使用 declare_parameter/get_parameter
    // 这里保持相同的参数名和默认值
    this->declare_parameter<std::string>(
      "pub_msg_to_moderator_topic_name", "/human_navigation/message/to_moderator");
    this->declare_parameter<std::string>(
      "sub_joint_state_topic_name", "/hsrb/joint_states");
    this->declare_parameter<std::string>(
      "pub_base_twist_topic_name", "/hsrb/command_velocity");
    this->declare_parameter<std::string>(
      "pub_base_trajectory_topic_name", "/hsrb/omni_base_controller/command");
    this->declare_parameter<std::string>(
      "pub_arm_trajectory_topic_name", "/hsrb/arm_trajectory_controller/command");
    this->declare_parameter<std::string>(
      "pub_gripper_trajectory_topic_name", "/hsrb/gripper_controller/command");

    std::string pub_msg_to_moderator_topic_name;
    std::string sub_joint_state_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_base_trajectory_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;

    this->get_parameter("pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name);
    this->get_parameter("sub_joint_state_topic_name",        sub_joint_state_topic_name);
    this->get_parameter("pub_base_twist_topic_name",         pub_base_twist_topic_name);
    this->get_parameter("pub_base_trajectory_topic_name",    pub_base_trajectory_topic_name);
    this->get_parameter("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name);
    this->get_parameter("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name);

    // 发布器/订阅器：与 ROS1 版本话题保持一致
    pub_msg_ = this->create_publisher<human_navigation_msgs::msg::HumanNaviMsg>(
      pub_msg_to_moderator_topic_name, 10);

    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      sub_joint_state_topic_name, 10,
      std::bind(&HSRKeyTeleopNode::jointStateCallback, this, std::placeholders::_1));

    pub_base_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
      pub_base_twist_topic_name, 10);
    pub_base_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      pub_base_trajectory_topic_name, 10);
    pub_arm_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      pub_arm_trajectory_topic_name, 10);
    pub_gripper_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      pub_gripper_trajectory_topic_name, 10);
    pub_guidance_msg_ = this->create_publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>(
      "/human_navigation/message/guidance_message", 10);

    // 原 ROS1: tf::TransformListener -> ROS2: tf2_ros::Buffer + TransformListener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    arm_lift_joint_pos1_  = 0.0;
    arm_lift_joint_pos2_  = 0.0;
    arm_flex_joint_pos_   = 0.0;
    wrist_flex_joint_pos_ = 0.0;
  }

  // run 对应原 ROS1::run 主循环，只是用 rclcpp::Rate 和 now()
  int run()
  {
    char c;

    // 获取控制台 raw 模式（与 ROS 无关，直接照搬）
    int kfd = 0;
    struct termios cooked;
    struct termios raw;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    showHelp();

    rclcpp::Rate loop_rate(40.0);  // 原 ROS1: ros::Rate loop_rate(40);

    const float linear_coef         = 0.2f;
    const float angular_coef        = 0.5f;

    float move_speed   = 1.0f;
    bool  is_hand_open = false;

    std::string arm_lift_joint_name   = "arm_lift_joint";
    std::string arm_flex_joint_name   = "arm_flex_joint";
    std::string wrist_flex_joint_name = "wrist_flex_joint";

    while (rclcpp::ok())
    {
      if (canReceive(kfd))
      {
        if (read(kfd, &c, 1) < 0)
        {
          perror("read():");
          return EXIT_FAILURE;
        }

        switch (c)
        {
          case KEYCODE_9:
          {
            sendMessage(MSG_GIVE_UP);
            break;
          }
          case KEYCODE_UP:
          {
            RCLCPP_DEBUG(this->get_logger(), "Go Forward");
            moveBaseTwist(+linear_coef * move_speed, 0.0, 0.0);
            break;
          }
          case KEYCODE_DOWN:
          {
            RCLCPP_DEBUG(this->get_logger(), "Go Backward");
            moveBaseTwist(-linear_coef * move_speed, 0.0, 0.0);
            break;
          }
          case KEYCODE_RIGHT:
          {
            RCLCPP_DEBUG(this->get_logger(), "Rotate Right");
            moveBaseTwist(0.0, 0.0, -angular_coef * move_speed);
            break;
          }
          case KEYCODE_LEFT:
          {
            RCLCPP_DEBUG(this->get_logger(), "Rotate Left");
            moveBaseTwist(0.0, 0.0, +angular_coef * move_speed);
            break;
          }
          case KEYCODE_SPACE:
          {
            RCLCPP_DEBUG(this->get_logger(), "Stop (twist)");
            moveBaseTwist(0.0, 0.0, 0.0);
            break;
          }
          case KEYCODE_U:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Left Forward");
            moveBaseJointTrajectory(+1.0, +1.0, +M_PI_4, 10.0);
            break;
          }
          case KEYCODE_I:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Forward");
            moveBaseJointTrajectory(+1.0, 0.0, 0.0, 10.0);
            break;
          }
          case KEYCODE_O:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Right Forward");
            moveBaseJointTrajectory(+1.0, -1.0, -M_PI_4, 10.0);
            break;
          }
          case KEYCODE_J:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Left");
            moveBaseJointTrajectory(0.0, +1.0, +M_PI_2, 10.0);
            break;
          }
          case KEYCODE_K:
          {
            RCLCPP_DEBUG(this->get_logger(), "Stop (joint trajectory)");
            moveBaseJointTrajectory(0.0, 0.0, 0.0, 0.5);
            break;
          }
          case KEYCODE_L:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Right");
            moveBaseJointTrajectory(0.0, -1.0, -M_PI_2, 10.0);
            break;
          }
          case KEYCODE_M:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Left Backward");
            moveBaseJointTrajectory(-1.0, +1.0, +M_PI_2 + M_PI_4, 10.0);
            break;
          }
          case KEYCODE_COMMA:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Backward");
            moveBaseJointTrajectory(-1.0, 0.0, +M_PI, 10.0);
            break;
          }
          case KEYCODE_PERIOD:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Right Backward");
            moveBaseJointTrajectory(-1.0, -1.0, -M_PI_2 - M_PI_4, 10.0);
            break;
          }
          case KEYCODE_Q:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Speed Up");
            move_speed *= 2.0f;
            if (move_speed > 2.0f) { move_speed = 2.0f; }
            break;
          }
          case KEYCODE_Z:
          {
            RCLCPP_DEBUG(this->get_logger(), "Move Speed Down");
            move_speed /= 2.0f;
            if (move_speed < 0.125f) { move_speed = 0.125f; }
            break;
          }
          case KEYCODE_Y:
          {
            RCLCPP_DEBUG(this->get_logger(), "Up Torso");
            operateArm(
              arm_lift_joint_name,
              0.69,
              std::max<int>(static_cast<int>(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1));
            break;
          }
          case KEYCODE_H:
          {
            RCLCPP_DEBUG(this->get_logger(), "Stop Torso");
            operateArm(
              arm_lift_joint_name,
              2.0 * arm_lift_joint_pos1_ - arm_lift_joint_pos2_,
              0.5);
            break;
          }
          case KEYCODE_N:
          {
            RCLCPP_DEBUG(this->get_logger(), "Down Torso");
            operateArm(
              arm_lift_joint_name,
              0.0,
              std::max<int>(static_cast<int>(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1));
            break;
          }
          case KEYCODE_A:
          {
            RCLCPP_DEBUG(this->get_logger(), "Rotate Arm - Vertical");
            operateArmFlex(0.0, -1.57);
            break;
          }
          case KEYCODE_B:
          {
            RCLCPP_DEBUG(this->get_logger(), "Rotate Arm - Upward");
            operateArmFlex(-0.785, -0.785);
            break;
          }
          case KEYCODE_C:
          {
            RCLCPP_DEBUG(this->get_logger(), "Rotate Arm - Horizontal");
            operateArmFlex(-1.57, 0.0);
            break;
          }
          case KEYCODE_D:
          {
            RCLCPP_DEBUG(this->get_logger(), "Rotate Arm - Downward");
            operateArmFlex(-2.2, 0.35);
            break;
          }
          case KEYCODE_G:
          {
            operateHand(is_hand_open);
            is_hand_open = !is_hand_open;
            break;
          }
          case KEYCODE_T:
          {
            sendGuidanceMessage("This is a test message.", "All");
            break;
          }
        }
      }

      rclcpp::spin_some(shared_from_this());
      loop_rate.sleep();
    }

    // 退出前恢复终端模式
    tcsetattr(kfd, TCSANOW, &cooked_);

    return EXIT_SUCCESS;
  }

private:
  // 原代码里的常量键值，直接照搬
  static constexpr char KEYCODE_9      = 0x39;
  static constexpr char KEYCODE_UP     = 0x41;
  static constexpr char KEYCODE_DOWN   = 0x42;
  static constexpr char KEYCODE_RIGHT  = 0x43;
  static constexpr char KEYCODE_LEFT   = 0x44;

  static constexpr char KEYCODE_A = 0x61;
  static constexpr char KEYCODE_B = 0x62;
  static constexpr char KEYCODE_C = 0x63;
  static constexpr char KEYCODE_D = 0x64;
  static constexpr char KEYCODE_G = 0x67;
  static constexpr char KEYCODE_H = 0x68;
  static constexpr char KEYCODE_I = 0x69;
  static constexpr char KEYCODE_J = 0x6a;
  static constexpr char KEYCODE_K = 0x6b;
  static constexpr char KEYCODE_L = 0x6c;
  static constexpr char KEYCODE_M = 0x6d;
  static constexpr char KEYCODE_N = 0x6e;
  static constexpr char KEYCODE_O = 0x6f;
  static constexpr char KEYCODE_Q = 0x71;
  static constexpr char KEYCODE_T = 0x74;
  static constexpr char KEYCODE_U = 0x75;
  static constexpr char KEYCODE_Y = 0x79;
  static constexpr char KEYCODE_Z = 0x7a;

  static constexpr char KEYCODE_COMMA  = 0x2c;
  static constexpr char KEYCODE_PERIOD = 0x2e;
  static constexpr char KEYCODE_SPACE  = 0x20;

  const std::string MSG_ARE_YOU_READY  = "Are_you_ready?";
  const std::string MSG_I_AM_READY     = "I_am_ready";
  const std::string MSG_GIVE_UP        = "Give_up";

  // 关节位置信息（与 ROS1 相同）
  double arm_lift_joint_pos1_{0.0};
  double arm_lift_joint_pos2_{0.0};
  double arm_flex_joint_pos_{0.0};
  double wrist_flex_joint_pos_{0.0};

  // 终端原始设置保存，用于退出时恢复
  struct termios cooked_{};

  // ROS2 通信成员
  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr          pub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr                  sub_joint_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                        pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr            pub_base_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr            pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr            pub_gripper_trajectory_;
  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pub_guidance_msg_;

  // tf2 监听器（原 ROS1: tf::TransformListener）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 关节状态回调：与 ROS1 等价
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
  {
    for (size_t i = 0; i < joint_state->name.size(); ++i)
    {
      if (joint_state->name[i] == "arm_lift_joint")
      {
        arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
        arm_lift_joint_pos1_ = joint_state->position[i];
      }
      if (joint_state->name[i] == "arm_flex_joint")
      {
        arm_flex_joint_pos_ = joint_state->position[i];
      }
      if (joint_state->name[i] == "wrist_flex_joint")
      {
        wrist_flex_joint_pos_ = joint_state->position[i];
      }
    }
  }

  void sendMessage(const std::string & message)
  {
    RCLCPP_INFO(this->get_logger(), "Send message:%s", message.c_str());
    human_navigation_msgs::msg::HumanNaviMsg human_navi_msg;
    human_navi_msg.message = message;
    human_navi_msg.detail  = "";
    pub_msg_->publish(human_navi_msg);
  }

  void moveBaseTwist(double linear_x, double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    pub_base_twist_->publish(twist);
  }

  void moveBaseJointTrajectory(double linear_x, double linear_y, double theta, double duration_sec)
  {
    // 原 ROS1: listener_.canTransform("/odom", "/base_footprint", ros::Time(0))
    // ROS2: 使用 tf2_ros::Buffer
    if (!tf_buffer_ || !tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero))
    {
      return;
    }

    geometry_msgs::msg::PointStamped basefootprint_2_target;
    geometry_msgs::msg::PointStamped odom_2_target;
    basefootprint_2_target.header.frame_id = "base_footprint";
    basefootprint_2_target.header.stamp    = this->now();
    basefootprint_2_target.point.x         = linear_x;
    basefootprint_2_target.point.y         = linear_y;

    tf_buffer_->transform(basefootprint_2_target, odom_2_target, "odom");

    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

    // 原 ROS1 使用 yaw + theta，这里保持行为：先计算 yaw，再加 theta
    double yaw = tf2::getYaw(transform.transform.rotation);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("odom_x");
    joint_trajectory.joint_names.push_back("odom_y");
    joint_trajectory.joint_names.push_back("odom_t");

    trajectory_msgs::msg::JointTrajectoryPoint omni_joint_point;
    omni_joint_point.positions = {
      odom_2_target.point.x,
      odom_2_target.point.y,
      yaw + theta
    };
    omni_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    joint_trajectory.points.push_back(omni_joint_point);
    pub_base_trajectory_->publish(joint_trajectory);
  }

  void operateArm(const double arm_lift_pos,
                  const double arm_flex_pos,
                  const double wrist_flex_pos,
                  const double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("arm_lift_joint");
    joint_trajectory.joint_names.push_back("arm_flex_joint");
    joint_trajectory.joint_names.push_back("arm_roll_joint");
    joint_trajectory.joint_names.push_back("wrist_flex_joint");
    joint_trajectory.joint_names.push_back("wrist_roll_joint");

    trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;
    arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, 0.0, wrist_flex_pos, 0.0};
    arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    joint_trajectory.points.push_back(arm_joint_point);
    pub_arm_trajectory_->publish(joint_trajectory);
  }

  void operateArm(const std::string & name,
                  const double position,
                  const double duration_sec)
  {
    if (name == "arm_lift_joint")
    {
      operateArm(position, arm_flex_joint_pos_, wrist_flex_joint_pos_, duration_sec);
    }
    else if (name == "arm_flex_joint")
    {
      operateArm(
        2.0 * arm_lift_joint_pos1_ - arm_lift_joint_pos2_,
        position,
        wrist_flex_joint_pos_,
        duration_sec);
    }
    else if (name == "wrist_flex_joint")
    {
      operateArm(
        2.0 * arm_lift_joint_pos1_ - arm_lift_joint_pos2_,
        arm_flex_joint_pos_,
        position,
        duration_sec);
    }
  }

  void operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos)
  {
    double duration = std::max(
      getDurationRot(arm_flex_pos, arm_flex_joint_pos_),
      getDurationRot(wrist_flex_pos, wrist_flex_joint_pos_));

    operateArm(
      2.0 * arm_lift_joint_pos1_ - arm_lift_joint_pos2_,
      arm_flex_pos,
      wrist_flex_pos,
      duration);
  }

  double getDurationRot(const double next_pos, const double current_pos)
  {
    return std::max<double>(std::abs(next_pos - current_pos) * 1.2, 1.0);
  }

  void operateHand(bool is_hand_open)
  {
    std::vector<std::string> joint_names{"hand_motor_joint"};
    std::vector<double> positions;

    if (is_hand_open)
    {
      RCLCPP_DEBUG(this->get_logger(), "Grasp");
      positions.push_back(-0.105);
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Open hand");
      positions.push_back(+1.239);
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    pub_gripper_trajectory_->publish(joint_trajectory);
  }

  void sendGuidanceMessage(const std::string & message, const std::string & displayType)
  {
    human_navigation_msgs::msg::HumanNaviGuidanceMsg guidanceMessage;
    guidanceMessage.message      = message;
    guidanceMessage.display_type = displayType;
    pub_guidance_msg_->publish(guidanceMessage);

    RCLCPP_INFO(
      this->get_logger(),
      "Send guide message: %s : %s",
      guidanceMessage.message.c_str(),
      guidanceMessage.display_type.c_str());
  }

  void showHelp()
  {
    // 与 ROS1 原版 help 文本一致
    std::puts("Operate by Keyboard");
    std::puts("---------------------------");
    std::puts("arrow keys : Move HSR");
    std::puts("space      : Stop HSR");
    std::puts("---------------------------");
    std::puts("Move HSR Linearly (1m)");
    std::puts("  u   i   o  ");
    std::puts("  j   k   l  ");
    std::puts("  m   ,   .  ");
    std::puts("---------------------------");
    std::puts("q/z : Increase/Decrease Moving Speed");
    std::puts("---------------------------");
    std::puts("y : Up   Torso");
    std::puts("h : Stop Torso");
    std::puts("n : Down Torso");
    std::puts("---------------------------");
    std::puts("a : Rotate Arm - Vertical");
    std::puts("b : Rotate Arm - Upward");
    std::puts("c : Rotate Arm - Horizontal");
    std::puts("d : Rotate Arm - Downward");
    std::puts("---------------------------");
    std::puts("g : Grasp/Open Hand");
    std::puts("---------------------------");
    std::puts("t : Send Test Message");
    std::puts(("9 : Send " + MSG_GIVE_UP).c_str());
  }

  static int canReceive(int fd)
  {
    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;

    return select(fd + 1, &fdset, nullptr, nullptr, &timeout);
  }
};

int main(int argc, char ** argv)
{
  // 原 ROS1: ros::init(argc, argv, "hsr_teleop_key");
  // ROS2: rclcpp::init / rclcpp::shutdown
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HSRKeyTeleopNode>();
  int ret = node->run();

  rclcpp::shutdown();
  return ret;
}

