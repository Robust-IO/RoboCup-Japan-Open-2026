#include <rclcpp/rclcpp.hpp>  // 原 ROS1: ros/ros.h -> ROS2: rclcpp 节点与日志接口
#include <geometry_msgs/msg/twist.hpp>  // 原 ROS1: geometry_msgs/Twist.h
#include <trajectory_msgs/msg/joint_trajectory.hpp>  // 原 ROS1: trajectory_msgs/JointTrajectory.h
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>  // 原 ROS1: trajectory_msgs/JointTrajectoryPoint.h

// 原 ROS1: <human_navigation/...> -> ROS2: human_navigation_msgs/msg/...
// 消息定义来自 ros2-competition-msgs 仓库中的 human_navigation_msgs 包
#include <human_navigation_msgs/msg/human_navi_object_info.hpp>
#include <human_navigation_msgs/msg/human_navi_destination.hpp>
#include <human_navigation_msgs/msg/human_navi_task_info.hpp>
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_avatar_status.hpp>
#include <human_navigation_msgs/msg/human_navi_object_status.hpp>

// 原 ROS1: tf/transform_listener.h -> ROS2: tf2_ros + tf2_geometry_msgs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <string>
#include <algorithm>

class HumanNavigationSampleCopy
{
private:
  // 与 ROS1 版本保持相同的状态机定义
  enum Step
  {
    Initialize,
    Ready,
    WaitTaskInfo,
    GuideForTakingObject,
    GuideForPlacement,
    //WaitTaskFinished,
    TaskFinished
  };

  enum class SpeechState
  {
    None,
    WaitingState,
    Speaking,
    Speakable
  };

  // human navigation message from/to the moderator
  // 与 ROS1 字符串常量完全一致，保证与 Unity 侧协议兼容
  const std::string MSG_ARE_YOU_READY      = "Are_you_ready?";
  const std::string MSG_TASK_SUCCEEDED     = "Task_succeeded";
  const std::string MSG_TASK_FAILED        = "Task_failed";
  const std::string MSG_TASK_FINISHED      = "Task_finished";
  const std::string MSG_GO_TO_NEXT_SESSION = "Go_to_next_session";
  const std::string MSG_MISSION_COMPLETE   = "Mission_complete";
  const std::string MSG_REQUEST            = "Guidance_request";
  const std::string MSG_SPEECH_STATE       = "Speech_state";
  const std::string MSG_SPEECH_RESULT      = "Speech_result";

  const std::string MSG_I_AM_READY        = "I_am_ready";
  const std::string MSG_GET_AVATAR_STATUS = "Get_avatar_status";
  const std::string MSG_GET_OBJECT_STATUS = "Get_object_status";
  const std::string MSG_GET_SPEECH_STATE  = "Get_speech_state";

  // 显示给受试者的引导面板类型，与 ROS1 完全一致
  const std::string DISPLAY_TYPE_ALL         = "All";
  const std::string DISPLAY_TYPE_ROBOT_ONLY  = "RobotOnly";
  const std::string DISPLAY_TYPE_AVATAR_ONLY = "AvatarOnly";
  const std::string DISPLAY_TYPE_NONE        = "None";

  int step{};
  SpeechState speechState{SpeechState::None};

  // 原 ROS1: tf::TransformListener listener_;
  // ROS2 中使用 tf2_ros::Buffer + TransformListener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool isStarted{false};
  bool isFinished{false};

  bool isTaskInfoReceived{false};
  bool isRequestReceived{false};

  // 原 ROS1: ros::Time -> ROS2: rclcpp::Time
  rclcpp::Time timePrevSpeechStateConfirmed;

  bool isSentGetAvatarStatus{false};
  bool isSentGetObjectStatus{false};

  human_navigation_msgs::msg::HumanNaviTaskInfo taskInfo;

  std::string guideMsg;

  human_navigation_msgs::msg::HumanNaviAvatarStatus avatarStatus;
  human_navigation_msgs::msg::HumanNaviObjectStatus objectStatus;

  std::string targetObjectName;

  double direction_target_object{0.0};
  double direction_target_direction{0.0};
  double avatar_timer{0.0};

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  int requestTime{0};
  bool request{false};
  std::string final_location;
  std::string initial_location;

  // 原 ROS1: ros::NodeHandle nodeHandle; -> ROS2: rclcpp::Node
  std::shared_ptr<rclcpp::Node> nodeHandle;

  // ROS2 订阅/发布器
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr subHumanNaviMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>::SharedPtr subTaskInfoMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>::SharedPtr subAvatarStatusMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>::SharedPtr subObjectStatusMsg;

  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr        pubHumanNaviMsg;
  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pubGuidanceMsg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                       pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr           pub_base_trajectory_;

  void init()
  {
    // 原 ROS1: step = Initialize; speechState = SpeechState::None; reset();
    step        = Initialize;
    speechState = SpeechState::None;
    reset();
  }

  void reset()
  {
    // 保持与 ROS1 版本相同的重置逻辑
    isStarted             = false;
    isFinished            = false;
    isTaskInfoReceived    = false;
    isRequestReceived     = false;
    isSentGetAvatarStatus = false;
    isSentGetObjectStatus = false;
  }

  // send humanNaviMsg to the moderator (Unity)
  // 原 ROS1: void sendMessage(ros::Publisher&, const std::string&) -> ROS2: 使用 rclcpp::Publisher SharedPtr
  void sendMessage(const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr & publisher,
                   const std::string & message)
  {
    human_navigation_msgs::msg::HumanNaviMsg human_navi_msg;
    human_navi_msg.message = message;
    human_navi_msg.detail  = "";
    publisher->publish(human_navi_msg);

    RCLCPP_INFO(nodeHandle->get_logger(), "Send message:%s", message.c_str());
  }

  // send guidance_message only
  // 原 ROS1: void sendGuidanceMessage(ros::Publisher&, ...) -> ROS2: 使用 rclcpp::Publisher SharedPtr
  void sendGuidanceMessage(
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr & publisher,
    const std::string & message,
    const std::string & displayType)
  {
    human_navigation_msgs::msg::HumanNaviGuidanceMsg guidanceMessage;
    guidanceMessage.message         = message;
    guidanceMessage.display_type    = displayType;
    guidanceMessage.source_language = "";  // Blank or ISO-639-1 language code
    guidanceMessage.target_language = "";
    publisher->publish(guidanceMessage);

    speechState = SpeechState::Speaking;

    RCLCPP_INFO(
      nodeHandle->get_logger(),
      "Send guide message: %s : %s",
      guidanceMessage.message.c_str(),
      guidanceMessage.display_type.c_str());
  }

  // receive humanNaviMsg from the moderator (Unity)
  // 原 ROS1: void messageCallback(const HumanNaviMsg::ConstPtr&) -> ROS2: ConstSharedPtr
  void messageCallback(const human_navigation_msgs::msg::HumanNaviMsg::ConstSharedPtr message)
  {
    RCLCPP_INFO(
      nodeHandle->get_logger(),
      "Subscribe message: %s : %s",
      message->message.c_str(),
      message->detail.c_str());

    if (message->message == MSG_ARE_YOU_READY)
    {
      isStarted = true;
      step      = Ready;
      sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);
    }
    else if (message->message == MSG_REQUEST)
    {
      if (isTaskInfoReceived && !isFinished)
      {
        isRequestReceived = true;
      }
    }
    else if (message->message == MSG_TASK_SUCCEEDED)
    {
      // 与 ROS1 一致，暂不处理
    }
    else if (message->message == MSG_TASK_FAILED)
    {
      init();
    }
    else if (message->message == MSG_TASK_FINISHED)
    {
      isFinished = true;
    }
    else if (message->message == MSG_GO_TO_NEXT_SESSION)
    {
      RCLCPP_INFO(nodeHandle->get_logger(), "Go to next session");
      init();
    }
    else if (message->message == MSG_MISSION_COMPLETE)
    {
      // ROS1 中注释掉 exit，这里同样保持不退出进程
    }
    else if (message->message == MSG_SPEECH_STATE)
    {
      if (message->detail == "Is_speaking")
      {
        speechState = SpeechState::Speaking;
      }
      else
      {
        speechState = SpeechState::Speakable;
      }
    }
    else if (message->message == MSG_SPEECH_RESULT)
    {
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Speech result: %s",
        message->detail.c_str());
    }
  }

  // 原 ROS1: moveBaseTwist(ros::Publisher&, double) -> ROS2: 使用 geometry_msgs::msg::Twist
  void moveBaseTwist(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr & publisher,
                     double rot)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = 0.0;
    twist.linear.y  = 0.0;
    twist.angular.z = rot;
    publisher->publish(twist);
  }

  // 原 ROS1: tf::TransformListener + geometry_msgs::PointStamped -> ROS2: tf2_ros::Buffer
  void moveBaseJointTrajectory(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher,
    double linear_x,
    double linear_y,
    double theta,
    double duration_sec)
  {
    if (!tf_buffer_ || !tf_buffer_->canTransform(
                         "odom",
                         "base_footprint",
                         tf2::TimePointZero))
    {
      return;
    }

    geometry_msgs::msg::PointStamped basefootprint_2_target;
    geometry_msgs::msg::PointStamped odom_2_target;
    basefootprint_2_target.header.frame_id = "base_footprint";
    basefootprint_2_target.header.stamp    = nodeHandle->now();
    basefootprint_2_target.point.x         = linear_x;
    basefootprint_2_target.point.y         = linear_y;

    tf_buffer_->transform(basefootprint_2_target, odom_2_target, "odom");

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("odom_x");
    joint_trajectory.joint_names.push_back("odom_y");
    joint_trajectory.joint_names.push_back("odom_t");

    trajectory_msgs::msg::JointTrajectoryPoint omni_joint_point;
    omni_joint_point.positions = {
      odom_2_target.point.x,
      odom_2_target.point.y,
      theta  // 原 ROS1: 使用 theta（而不是 yaw+theta），保持行为一致
    };
    omni_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    joint_trajectory.points.push_back(omni_joint_point);
    publisher->publish(joint_trajectory);
  }

  std::string getFinalDestination(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr & message)
  {
    taskInfo = *message;

    double dx = taskInfo.destination.position.x;
    double dy = taskInfo.destination.position.y;
    double dz = taskInfo.destination.position.z;

    int   i                     = static_cast<int>(taskInfo.furniture.size());
    int   index_of_nearest_furniture = 0;
    float min_distance_furniture = 999999999.0f;

    for (int j = 0; j < i - 1; ++j)
    {
      float distance =
        std::sqrt(
          std::pow(taskInfo.destination.position.x - taskInfo.furniture[j].position.x, 2) +
          std::pow(taskInfo.destination.position.y - taskInfo.furniture[j].position.y, 2) +
          std::pow(taskInfo.destination.position.z - taskInfo.furniture[j].position.z, 2));

      if (distance < min_distance_furniture)
      {
        index_of_nearest_furniture = j;
        min_distance_furniture     = distance;
      }

      double fx_min = taskInfo.furniture[i].position.x - taskInfo.destination.size.x / 2.0;
      double fx_max = taskInfo.furniture[i].position.x + taskInfo.destination.size.x / 2.0;
      double fy_min = taskInfo.furniture[i].position.y - taskInfo.destination.size.y / 2.0;
      double fy_max = taskInfo.furniture[i].position.y + taskInfo.destination.size.y / 2.0;
      double fz_min = taskInfo.furniture[i].position.z;
      double fz_max = taskInfo.furniture[i].position.z + taskInfo.destination.size.z;

      bool within = dx <= fx_max && dx >= fx_min && dy <= fy_max && dy >= fy_min;

      if (within)
      {
        std::string furniture_name = taskInfo.furniture[j].name;
        std::replace(furniture_name.begin(), furniture_name.end(), '_', ' ');

        if (dx >= fx_min && dx <= fx_max &&
            dy >= fy_min && dy <= fy_max &&
            dz >= fz_min && dz <= fz_max)
        {
          return "Please place it where the robot is pointing, inside the " + furniture_name;
        }

        if (dz >= fz_min && dz <= fz_max)
        {
          return "Please place it where the robot is pointing, on the " + furniture_name;
        }

        if (dz < fz_max)
        {
          return "Please place it where the robot is pointing, under the " + furniture_name;
        }
      }
    }

    std::string nearest_furniture_name = taskInfo.furniture[index_of_nearest_furniture].name;
    std::replace(nearest_furniture_name.begin(), nearest_furniture_name.end(), '_', ' ');

    return "Please place it where the robot is pointing, near the " + nearest_furniture_name;
  }

  std::string getInitialDestination(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr & message)
  {
    taskInfo = *message;

    double dx = taskInfo.target_object.position.x;
    double dy = taskInfo.target_object.position.y;
    double dz = taskInfo.target_object.position.z;

    int   index_of_nearest_furniture = 0;
    float min_distance               = 999999999.0f;

    int i = static_cast<int>(taskInfo.furniture.size());
    for (int j = 0; j < i - 1; ++j)
    {
      double fx = taskInfo.furniture[j].position.x;
      double fy = taskInfo.furniture[j].position.y;
      double fz = taskInfo.furniture[j].position.z;

      float distance = std::sqrt(
        std::pow(dx - fx, 2) +
        std::pow(dy - fy, 2) +
        std::pow(dz - fz, 2));

      if (distance < min_distance)
      {
        index_of_nearest_furniture = j;
        min_distance               = distance;
      }
    }

    std::string nearest_furniture = taskInfo.furniture[index_of_nearest_furniture].name;
    std::replace(nearest_furniture.begin(), nearest_furniture.end(), '_', ' ');

    int   index_of_nearest_non = 0;
    float min_distance_non     = 999999999.0f;

    int k = static_cast<int>(taskInfo.non_target_objects.size());
    for (int j = 0; j < k - 1; ++j)
    {
      double fx = taskInfo.non_target_objects[j].position.x;
      double fy = taskInfo.non_target_objects[j].position.y;
      double fz = taskInfo.non_target_objects[j].position.z;

      float distance = std::sqrt(
        std::pow(dx - fx, 2) +
        std::pow(dy - fy, 2) +
        std::pow(dz - fz, 2));

      if (distance < min_distance_non)
      {
        index_of_nearest_non = j;
        min_distance_non     = distance;
      }
    }

    std::string nearest_non_target = taskInfo.non_target_objects[index_of_nearest_non].name;
    std::replace(nearest_non_target.begin(), nearest_non_target.end(), '_', ' ');

    if (min_distance_non < 0.4)
    {
      return "Please take " + targetObjectName +
             ", you will find it where the robot is pointing, near the " + nearest_furniture +
             ",and be careful it's near to the " + nearest_non_target;
    }
    else
    {
      return "Please take " + targetObjectName +
             ", you will find it where the robot is pointing, near the " + nearest_furniture;
    }
  }

  // receive taskInfo from the moderator (Unity)
  // 原 ROS1: taskInfoMessageCallback(const HumanNaviTaskInfo::ConstPtr&) -> ROS2: ConstSharedPtr
  void taskInfoMessageCallback(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr message)
  {
    taskInfo = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe task info message:" << std::endl <<
      "Environment ID: " << taskInfo.environment_id);

    targetObjectName        = taskInfo.target_object.name;
    direction_target_object =
      std::atan(taskInfo.target_object.position.y / taskInfo.target_object.position.x);
    direction_target_direction =
      std::atan(taskInfo.destination.position.y / taskInfo.destination.position.x);

    final_location   = getFinalDestination(message);
    initial_location = getInitialDestination(message);

    isTaskInfoReceived = true;
  }

  // receive avatar status
  void avatarStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviAvatarStatus::ConstSharedPtr message)
  {
    avatarStatus = *message;

    // 原 ROS1: 直接将 geometry_msgs::Pose 等打印到流中
    // 在 ROS2 中，geometry_msgs::msg::Pose 没有 operator<<，为避免编译错误，仅打印关键字段
    RCLCPP_INFO(
      nodeHandle->get_logger(),
      "Subscribe avatar status message: objctInLeftHand=%s, objectInRightHand=%s, "
      "isTargetObjectInLeftHand=%d, isTargetObjectInRightHand=%d",
      avatarStatus.object_in_left_hand.c_str(),
      avatarStatus.object_in_right_hand.c_str(),
      static_cast<int>(avatarStatus.is_target_object_in_left_hand),
      static_cast<int>(avatarStatus.is_target_object_in_right_hand));

    isSentGetAvatarStatus = false;
  }

  void objectStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviObjectStatus::ConstSharedPtr message)
  {
    objectStatus = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe object status message:" << std::endl <<
      "Target object size of non_target_objects: " <<
      taskInfo.non_target_objects.size());

    int numOfNonTargetObjects = static_cast<int>(taskInfo.non_target_objects.size());
    std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
    std::cout << "Non-target objects:" << std::endl;
    for (int i = 0; i < numOfNonTargetObjects; ++i)
    {
      std::cout << taskInfo.non_target_objects[i].name << std::endl;
    }

    isSentGetObjectStatus = false;
  }

  // 原 ROS1: bool speakGuidanceMessage(ros::Publisher, ros::Publisher, ...) ->
  // ROS2: 使用 Publisher SharedPtr，时间改为 nodeHandle->now()
  bool speakGuidanceMessage(
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr & pubHumanNaviMsgLocal,
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr & pubGuidanceMsgLocal,
    const std::string & message,
    int interval = 1)
  {
    if (speechState == SpeechState::Speakable)
    {
      sendGuidanceMessage(pubGuidanceMsgLocal, message, DISPLAY_TYPE_ALL);
      speechState = SpeechState::None;
      return true;
    }
    else if (speechState == SpeechState::None || speechState == SpeechState::Speaking)
    {
      if ((timePrevSpeechStateConfirmed.seconds() + interval) < nodeHandle->now().seconds())
      {
        sendMessage(pubHumanNaviMsgLocal, MSG_GET_SPEECH_STATE);
        timePrevSpeechStateConfirmed = nodeHandle->now();
        speechState                  = SpeechState::Speakable;
      }
    }

    return false;
  }

public:
  int run(int argc, char ** argv)
  {
    (void)argc;
    (void)argv;

    // 类内部创建 Node，接口与前一个 ROS2 节点类似
    nodeHandle = std::make_shared<rclcpp::Node>("human_navi_sample_copy");

    // 初始化 tf2 Buffer/Listener（替代 ROS1 的 tf::TransformListener）
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(nodeHandle->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::Rate loopRate(10.0);  // 原 ROS1: ros::Rate loopRate(10);

    init();

    RCLCPP_INFO(nodeHandle->get_logger(), "Human Navi sample copy start!");

    // ROS2 订阅/发布器创建，对应 ROS1 NodeHandle.subscribe/advertise
    subHumanNaviMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_robot",
      100,
      std::bind(&HumanNavigationSampleCopy::messageCallback, this, std::placeholders::_1));
    subTaskInfoMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>(
      "/human_navigation/message/task_info",
      1,
      std::bind(&HumanNavigationSampleCopy::taskInfoMessageCallback, this, std::placeholders::_1));
    subAvatarStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>(
      "/human_navigation/message/avatar_status",
      1,
      std::bind(&HumanNavigationSampleCopy::avatarStatusMessageCallback, this, std::placeholders::_1));
    subObjectStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>(
      "/human_navigation/message/object_status",
      1,
      std::bind(&HumanNavigationSampleCopy::objectStatusMessageCallback, this, std::placeholders::_1));

    pubHumanNaviMsg = nodeHandle->create_publisher<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_moderator",
      10);
    pubGuidanceMsg = nodeHandle->create_publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>(
      "/human_navigation/message/guidance_message",
      10);
    pub_base_twist_ = nodeHandle->create_publisher<geometry_msgs::msg::Twist>(
      "/hsrb/command_velocity",
      10);
    pub_base_trajectory_ = nodeHandle->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/hsrb/omni_base_controller/command",
      10);

    timePrevSpeechStateConfirmed = nodeHandle->now();

    rclcpp::Time time = nodeHandle->now();

    while (rclcpp::ok())
    {
      switch (step)
      {
        case Initialize:
        {
          reset();
          RCLCPP_INFO(nodeHandle->get_logger(), "##### Initialized ######");
          step++;
          break;
        }
        case Ready:
        {
          if (isStarted)
          {
            step++;
            RCLCPP_INFO(nodeHandle->get_logger(), "Task start");
          }
          break;
        }
        case WaitTaskInfo:
        {
          if (isTaskInfoReceived)
          {
            RCLCPP_INFO(nodeHandle->get_logger(), "Task Received");
            isRequestReceived = true;
            avatar_timer      = nodeHandle->now().seconds();
            step++;
          }
          break;
        }
        case GuideForTakingObject:
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "GuideForTakingObject");

          if (nodeHandle->now().seconds() - avatar_timer > 0.100)
          {
            sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
            avatar_timer = nodeHandle->now().seconds();

            if (static_cast<bool>(avatarStatus.is_target_object_in_left_hand) ||
                static_cast<bool>(avatarStatus.is_target_object_in_right_hand))
            {
              guideMsg = "Please Keep holding the Object";
              while (!speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                RCLCPP_INFO(nodeHandle->get_logger(), "still waiting");
                rclcpp::spin_some(nodeHandle);
              }

              step             = GuideForPlacement;
              isRequestReceived = true;
              break;
            }
            else if (!avatarStatus.object_in_left_hand.empty() ||
                     !avatarStatus.object_in_right_hand.empty())
            {
              guideMsg = "This is not the object, Please try again!";
              while (!speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                RCLCPP_INFO(nodeHandle->get_logger(), "still waiting_2");
                rclcpp::spin_some(nodeHandle);
              }
              break;
            }
          }

          if (isRequestReceived)
          {
            moveBaseJointTrajectory(pub_base_trajectory_, 0.0, 0.0, direction_target_object, 5.0);

            guideMsg = initial_location;

            if (speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
            {
              time             = nodeHandle->now();
              isRequestReceived = false;
            }
          }
          break;
        }
        case GuideForPlacement:
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "GuideForPlacement");

          if (isRequestReceived)
          {
            moveBaseJointTrajectory(
              pub_base_trajectory_,
              0.0, 0.0,
              direction_target_direction,
              5.0);

            guideMsg = final_location;

            if (speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
            {
              isRequestReceived = false;
            }
          }

          break;
        }
        case TaskFinished:
        {
          reset();
          break;
        }
      }

      rclcpp::spin_some(nodeHandle);
      loopRate.sleep();
    }

    return 0;
  }
};

int main(int argc, char ** argv)
{
  // 原 ROS1: ros::init(argc, argv, "human_navi_sample");
  // ROS2: 使用 rclcpp::init / shutdown
  rclcpp::init(argc, argv);

  HumanNavigationSampleCopy humanNaviSample;
  int ret = humanNaviSample.run(argc, argv);

  rclcpp::shutdown();
  return ret;
}

