#include <rclcpp/rclcpp.hpp>  // 原 ROS1: ros/ros.h -> ROS2: rclcpp 节点与日志接口

// 原 ROS1: <human_navigation/...> -> ROS2: human_navigation_msgs/msg/...
// 消息定义来自 ros2-competition-msgs 仓库中的 human_navigation_msgs 包
#include <human_navigation_msgs/msg/human_navi_object_info.hpp>
#include <human_navigation_msgs/msg/human_navi_destination.hpp>
#include <human_navigation_msgs/msg/human_navi_task_info.hpp>
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_avatar_status.hpp>
#include <human_navigation_msgs/msg/human_navi_object_status.hpp>

#include <rosidl_runtime_cpp/traits.hpp>  // 为了使用 to_yaml 打印消息内容（优化调试，逻辑不变）

#include <string>
#include <iostream>

class HumanNavigationSampleOrigin
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
    WaitTaskFinished,
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

  // 原 ROS1: ros::NodeHandle nodeHandle; -> ROS2: rclcpp::Node
  std::shared_ptr<rclcpp::Node> nodeHandle;

  // ROS2 订阅/发布器
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr subHumanNaviMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>::SharedPtr subTaskInfoMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>::SharedPtr subAvatarStatusMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>::SharedPtr subObjectStatusMsg;

  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr        pubHumanNaviMsg;
  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pubGuidanceMsg;

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
    // ROS1 中 detail 默认空，这里保持为空字符串
    human_navi_msg.detail = "";
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
      // 原始版本未做处理，这里保持不动
    }
    else if (message->message == MSG_TASK_FAILED)
    {
      // 原始版本未做处理，这里保持不动
    }
    else if (message->message == MSG_TASK_FINISHED)
    {
      isFinished = true;
    }
    else if (message->message == MSG_GO_TO_NEXT_SESSION)
    {
      RCLCPP_INFO(nodeHandle->get_logger(), "Go to next session");
      step = Initialize;
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

  // receive taskInfo from the moderator (Unity)
  // 原 ROS1: taskInfoMessageCallback(const HumanNaviTaskInfo::ConstPtr&) -> ROS2: ConstSharedPtr
  void taskInfoMessageCallback(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr message)
  {
    taskInfo = *message;

    // 为了便于调试，使用 to_yaml 打印内容（逻辑行为与原来的“打印对象”等价）
    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe task info message:" << std::endl <<
      "Environment ID: " << taskInfo.environment_id << std::endl <<
      "Target object: " << std::endl <<
        human_navigation_msgs::msg::to_yaml(taskInfo.target_object) <<
      "Destination: " << std::endl <<
        human_navigation_msgs::msg::to_yaml(taskInfo.destination));

    int numOfNonTargetObjects = static_cast<int>(taskInfo.non_target_objects.size());
    std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
    std::cout << "Non-target objects:" << std::endl;
    for (int i = 0; i < numOfNonTargetObjects; ++i)
    {
      std::cout << human_navigation_msgs::msg::to_yaml(taskInfo.non_target_objects[i]) << std::endl;
    }

    int numOfFurniture = static_cast<int>(taskInfo.furniture.size());
    std::cout << "Number of furniture: " << numOfFurniture << std::endl;
    std::cout << "Furniture objects:" << std::endl;
    for (int i = 0; i < numOfFurniture; ++i)
    {
      std::cout << human_navigation_msgs::msg::to_yaml(taskInfo.furniture[i]) << std::endl;
    }

    isTaskInfoReceived = true;
  }

  void avatarStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviAvatarStatus::ConstSharedPtr message)
  {
    avatarStatus = *message;

    // 原 ROS1 直接用流输出 Pose，这里同样改为用 to_yaml 打印主要字段
    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe avatar status message:" << std::endl <<
      "Head: "      << std::endl << geometry_msgs::msg::to_yaml(avatarStatus.head) <<
      "LeftHand: "  << std::endl << geometry_msgs::msg::to_yaml(avatarStatus.left_hand) <<
      "rightHand: " << std::endl << geometry_msgs::msg::to_yaml(avatarStatus.right_hand) <<
      "objctInLeftHand: "   << avatarStatus.object_in_left_hand << std::endl <<
      "objectInRightHand: " << avatarStatus.object_in_right_hand << std::endl <<
      "isTargetObjectInLeftHand: " << std::boolalpha
        << static_cast<bool>(avatarStatus.is_target_object_in_left_hand) << std::endl <<
      "isTargetObjectInRightHand: " << std::boolalpha
        << static_cast<bool>(avatarStatus.is_target_object_in_right_hand) << std::endl);

    isSentGetAvatarStatus = false;
  }

  void objectStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviObjectStatus::ConstSharedPtr message)
  {
    objectStatus = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe object status message:" << std::endl <<
      "Target object: " << std::endl <<
        human_navigation_msgs::msg::to_yaml(taskInfo.target_object));

    int numOfNonTargetObjects = static_cast<int>(taskInfo.non_target_objects.size());
    std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
    std::cout << "Non-target objects:" << std::endl;
    for (int i = 0; i < numOfNonTargetObjects; ++i)
    {
      std::cout << human_navigation_msgs::msg::to_yaml(taskInfo.non_target_objects[i]) << std::endl;
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
        speechState                  = SpeechState::WaitingState;
      }
    }

    return false;
  }

public:
  int run(int argc, char ** argv)
  {
    (void)argc;
    (void)argv;

    // 类内部创建 Node
    nodeHandle = std::make_shared<rclcpp::Node>("human_navi_sample_origin");

    rclcpp::Rate loopRate(10.0);  // 原 ROS1: ros::Rate loopRate(10);

    init();

    RCLCPP_INFO(nodeHandle->get_logger(), "Human Navi sample (origin) start!");

    // ROS2 订阅/发布器创建，对应 ROS1 NodeHandle.subscribe/advertise
    subHumanNaviMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_robot",
      100,
      std::bind(&HumanNavigationSampleOrigin::messageCallback, this, std::placeholders::_1));
    subTaskInfoMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>(
      "/human_navigation/message/task_info",
      1,
      std::bind(&HumanNavigationSampleOrigin::taskInfoMessageCallback, this, std::placeholders::_1));
    subAvatarStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>(
      "/human_navigation/message/avatar_status",
      1,
      std::bind(&HumanNavigationSampleOrigin::avatarStatusMessageCallback, this, std::placeholders::_1));
    subObjectStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>(
      "/human_navigation/message/object_status",
      1,
      std::bind(&HumanNavigationSampleOrigin::objectStatusMessageCallback, this, std::placeholders::_1));

    pubHumanNaviMsg = nodeHandle->create_publisher<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_moderator",
      10);
    pubGuidanceMsg = nodeHandle->create_publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>(
      "/human_navigation/message/guidance_message",
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
            sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);
            RCLCPP_INFO(nodeHandle->get_logger(), "Task start");
          }
          break;
        }
        case WaitTaskInfo:
        {
          if (isTaskInfoReceived)
          {
            step++;
          }
          break;
        }
        case GuideForTakingObject:
        {
          if (isRequestReceived)
          {
            isRequestReceived = false;
          }

          std::string targetObjectName;
          if (taskInfo.target_object.name.find("empty_plastic_bottle") != std::string::npos)
          {
            targetObjectName = "an empty plastic bottle ";
          }
          else
          {
            targetObjectName = "a cup ";
          }

          std::string locationName;
          if (taskInfo.target_object.position.x > 0.0)
          {
            locationName = "on a table.";
          }
          else
          {
            locationName = "next to the kitchen sink.";
          }

          guideMsg = "Please take " + targetObjectName + locationName;

          if (speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
          {
            time = nodeHandle->now();
            step++;
          }
          break;
        }
        case GuideForPlacement:
        {
          if (isRequestReceived)
          {
            if (speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
            {
              isRequestReceived = false;
            }
          }

          {
            int WaitTime = 5;
            if (time.seconds() + WaitTime < nodeHandle->now().seconds())
            {
              std::string destinationName;
              if (taskInfo.destination.position.z < 1.0)
              {
                destinationName = "a trash can on the left.";
              }
              else
              {
                destinationName = "the second cabinet from the right.";
              }
              guideMsg = "Put it in " + destinationName;

              if (speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                time = nodeHandle->now();
                step++;
              }
            }
          }

          break;
        }
        case WaitTaskFinished:
        {
          if (isFinished)
          {
            RCLCPP_INFO(nodeHandle->get_logger(), "Task finished");
            step++;
            break;
          }

          if (isRequestReceived)
          {
            bool isSpeaked;
            if ((static_cast<int>(nodeHandle->now().seconds()) % 2) > 0)
            {
              isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg);
            }
            else
            {
              isSpeaked = speakGuidanceMessage(
                pubHumanNaviMsg,
                pubGuidanceMsg,
                "You can find the wall cabinet above the kitchen sink.");
            }

            if (isSpeaked)
            {
              isRequestReceived = false;
            }
          }

          {
            int WaitTime = 5;
            if (time.seconds() + WaitTime < nodeHandle->now().seconds())
            {
              if (!isSentGetAvatarStatus && !isSentGetObjectStatus)
              {
                sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
                sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
                isSentGetAvatarStatus = true;
                isSentGetObjectStatus = true;
                time = nodeHandle->now();
              }
            }
          }

          break;
        }
        case TaskFinished:
        {
          // Wait MSG_GO_TO_NEXT_SESSION or MSG_MISSION_COMPLETE
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

  HumanNavigationSampleOrigin humanNaviSample;
  int ret = humanNaviSample.run(argc, argv);

  rclcpp::shutdown();
  return ret;
}

