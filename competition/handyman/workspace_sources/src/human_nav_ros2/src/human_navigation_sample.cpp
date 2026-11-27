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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <iomanip>
#include <string>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <cctype>
#include <iterator>
#include <utility>
#include <vector>
#include <deque>
#include <unordered_set>
#include <unordered_map>

class HumanNavigationSample
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
    // WaitTaskFinished,  // 在 2025 版本中未使用，保持注释
    TaskFinished
  };

  enum class SpeechState
  {
    None,
    WaitingState,
    Speaking,
    Speakable
  };

  enum class GuidancePriority
  {
    Automatic = 0,
    Requested = 1,
    Transition = 2,
    Error = 3
  };

  struct PendingGuidance
  {
    std::string message;
    std::string display_type;
    GuidancePriority priority;
  };

  enum class PlacementCheckState
  {
    Idle,
    DropDelay,
    WaitingObjectStatus,
    SuccessGrace
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
  int questcounter{};  // 2025 版本新增：用于控制距离提示频率，ROS2 保留此逻辑
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

  // 原 ROS1: NodeHandle 成员上直接调用 subscribe/advertise
  // ROS2: 使用 rclcpp::Subscription/Publisher 成员
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr subHumanNaviMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>::SharedPtr subTaskInfoMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>::SharedPtr subAvatarStatusMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>::SharedPtr subObjectStatusMsg;

  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr pubHumanNaviMsg;
  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pubGuidanceMsg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_base_trajectory_;

  /// 抓取阶段骨架句；周期性距离行在此基础上拼接
  std::string polished_pick_base_;
  /// 放置阶段骨架句
  std::string polished_place_base_;
  /// TaskInfo 到达后待主循环中 polish（避免在订阅回调里 spin_until_future_complete 死锁）
  bool pending_polish_{false};

  /// 方位提示节流：上次发送时间(秒)、间隔秒数（规则要求≤15条，不再实时距离）
  double last_direction_hint_sec_{0.0};
  double direction_hint_interval_sec_{10.0};
  /// 抓取正确提示后，字幕保持到该时间点（秒）；期间禁止新字幕覆盖
  double subtitle_hold_until_sec_{0.0};
  std::string last_wrong_object_name_;
  bool wrong_object_hold_active_{false};
  bool target_object_was_held_{false};
  bool placement_prompt_locked_{false};
  bool placement_seeking_target_{false};
  bool awaiting_requested_avatar_status_{false};
  PlacementCheckState placement_check_state_{PlacementCheckState::Idle};
  double placement_check_deadline_sec_{0.0};
  double object_status_request_sec_{0.0};
  bool completion_guidance_pending_{false};
  double drop_check_delay_sec_{1.0};
  double object_status_timeout_sec_{1.0};
  double placement_success_grace_sec_{2.0};
  double initial_region_radius_{0.5};
  double destination_region_tolerance_{0.1};

  // 所有 guidance 必须经过此队列。自动提示最多占 12 条，剩余 3 条由
  // 主动请求、阶段转换和抓错提示共享；全场硬上限为 15 条。
  static constexpr size_t MAX_GUIDANCE_COUNT = 15;
  static constexpr size_t MAX_AUTOMATIC_GUIDANCE_COUNT = 12;
  std::deque<PendingGuidance> guidance_queue_;
  size_t guidance_sent_count_{0};
  size_t automatic_guidance_sent_count_{0};
  double last_guidance_sent_sec_{0.0};
  double guidance_min_interval_sec_{1.0};
  double speech_state_poll_interval_sec_{0.5};
  double speech_state_timeout_sec_{2.0};

  std::string getDirectionHint(
    double av_x, double av_y,
    double target_x, double target_y,
    const geometry_msgs::msg::Quaternion & body_orient) const;
  static std::string toReadableName(const std::string & name);
  static std::string toLowerAscii(std::string s);
  static bool isLargeLandmarkName(const std::string & readable_name_lower);
  static bool isRefrigeratorName(const std::string & readable_name_lower);
  static std::string extractThePhraseLower(const std::string & phrase);
  bool isFusedSlotName(const std::string & readable_name) const;
  std::string normalizeKnownFusionPhrase(std::string text) const;
  /// 基于 getFurnitureRelation 的空间判断，返回 "on/in/near the X"（仅来自 furniture，不掺入 non_target）
  std::string getRelationPhraseForPosition(double dx, double dy, double dz,
                                          double size_x, double size_y, double size_z) const;
  std::string nearestFurnitureName(double x, double y, double z) const;
  /// 抓取阶段 on 槽位：目标物体下方大型不可交互（先 0-2m，失败再扩大范围）
  std::string buildPickOnSlotPhrase(double px, double py, double pz, const std::unordered_set<std::string> & banned_lower) const;
  /// 目标点下的家具介词短语：优先 on/in（getRelationPhraseForPosition），否则 on the + 最近家具
  std::string buildOnSlotPhrase(
    double px, double py, double pz, double sx, double sy, double sz,
    const std::unordered_set<std::string> & banned_lower) const;
  std::string nearestNearLandmarkName(
    double x, double y, double z,
    double min_distance_m,
    double max_distance_m,
    const std::unordered_set<std::string> & banned_lower) const;
  std::string nearestNearbyObjectName(
    double x, double y, double z,
    double max_distance_m,
    const std::unordered_set<std::string> & banned_lower) const;
  std::string buildStrictPickTemplate() const;
  std::string buildStrictPlaceTemplate() const;

  static std::string truncateUtf8(const std::string & s, size_t max_chars);
  static std::string escapeJsonString(const std::string & s);
  std::string buildContextJson() const;
  std::string polishGuidance(const std::string & draft, const std::string & phase, const std::string & context_json);
  void init()
  {
    // 原 ROS1: step = Initialize; speechState = SpeechState::None; reset();
    step = Initialize;
    speechState = SpeechState::None;
    reset();
  }

  void reset()
  {
    // 保持与 ROS1 2025 版本相同的重置逻辑
    isStarted             = false;
    isFinished            = false;
    isTaskInfoReceived    = false;
    isRequestReceived     = false;
    isSentGetAvatarStatus = false;
    isSentGetObjectStatus = false;
    questcounter          = 0;

    // 重置 avatar 状态
    avatarStatus.object_in_left_hand = "";
    avatarStatus.object_in_right_hand = "";
    avatarStatus.is_target_object_in_left_hand = false;
    avatarStatus.is_target_object_in_right_hand = false;

    polished_pick_base_.clear();
    polished_place_base_.clear();
    pending_polish_ = false;
    last_direction_hint_sec_ = 0.0;
    subtitle_hold_until_sec_ = 0.0;
    last_wrong_object_name_.clear();
    wrong_object_hold_active_ = false;
    target_object_was_held_ = false;
    placement_prompt_locked_ = false;
    placement_seeking_target_ = false;
    awaiting_requested_avatar_status_ = false;
    placement_check_state_ = PlacementCheckState::Idle;
    placement_check_deadline_sec_ = 0.0;
    object_status_request_sec_ = 0.0;
    completion_guidance_pending_ = false;
    guidance_queue_.clear();
    guidance_sent_count_ = 0;
    automatic_guidance_sent_count_ = 0;
    last_guidance_sent_sec_ = 0.0;
    speechState = SpeechState::None;
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

  // 唯一的底层 guidance 发布函数；业务代码只能调用 queueGuidance()。
  // 原 ROS1: void sendGuidanceMessage(ros::Publisher&, ...) -> ROS2: 使用 rclcpp::Publisher SharedPtr
  void sendGuidanceMessage(
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr & publisher,
    const std::string & message,
    const std::string & displayType)
  {
    const std::string normalized_message = normalizeKnownFusionPhrase(message);
    const std::string safe_message = truncateUtf8(normalized_message, 400);

    human_navigation_msgs::msg::HumanNaviGuidanceMsg guidanceMessage;
    guidanceMessage.message         = safe_message;
    guidanceMessage.display_type    = displayType;
    guidanceMessage.source_language = "";  // Blank or ISO-639-1 language code
    guidanceMessage.target_language = "";

    publisher->publish(guidanceMessage);

    RCLCPP_INFO(
      nodeHandle->get_logger(),
      "Send guide message: %s : %s",
      guidanceMessage.message.c_str(),
      guidanceMessage.display_type.c_str());
    if (normalized_message != safe_message) {
      RCLCPP_WARN(
        nodeHandle->get_logger(),
        "Guidance truncated to <=400 Unicode scalars for competition limit");
    }
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
    else if (message->message == MSG_REQUEST)  // Guidance_request
    {
      if (isTaskInfoReceived && !isFinished)
      {
        isRequestReceived = true;
        // 主动请求必须使用请求之后返回的 AvatarStatus，不能读取旧缓存或省略距离。
        awaiting_requested_avatar_status_ = true;
        sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
        isSentGetAvatarStatus = true;
      }
    }
    else if (message->message == MSG_TASK_SUCCEEDED)
    {
      finishWithCompletionGuidance();
    }
    else if (message->message == MSG_TASK_FAILED)
    {
      init();
    }
    else if (message->message == MSG_TASK_FINISHED)
    {
      finishWithCompletionGuidance();
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
    if (!tf_buffer_)
    {
      return;
    }
    try
    {
      // 使用 TimePointZero 取“最新可用”的 TF，避免 nodeHandle->now() 领先于仿真 TF 导致 ExtrapolationException
      if (!tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero))
      {
        return;
      }

      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

      geometry_msgs::msg::PointStamped basefootprint_2_target;
      geometry_msgs::msg::PointStamped odom_2_target;
      basefootprint_2_target.header.frame_id = "base_footprint";
      basefootprint_2_target.header.stamp    = transform.header.stamp;  // 使用 TF 时间，避免外推
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
    catch (const tf2::TransformException & e)
    {
      RCLCPP_DEBUG(
        nodeHandle->get_logger(),
        "moveBaseJointTrajectory: TF not available (%s), skipping.",
        e.what());
    }
  }

  // 将 target_object.name（如 empty_plastic_bottle_0）转为给受试者看的自然语言
  static std::string getHumanReadableTargetName(const std::string & name)
  {
    if (name.find("empty_plastic_bottle") != std::string::npos)
    {
      return "an empty plastic bottle";
    }
    if (name.find("cup") != std::string::npos)
    {
      return "a cup";
    }
    // 默认：统一走 toReadableName，清理 Unity 后缀噪声（如 _C01 / _C_02 / _A）
    return "the " + toReadableName(name);
  }

  // 下方 getFinalDestination/getInitialDestination/getFurnitureRelation 基本照搬 2025 逻辑
  std::string getFurnitureRelation(const std::string & furniture_name,
                                   double dx, double dy, double dz,
                                   double fx_min, double fx_max,
                                   double fy_min, double fy_max,
                                   double fz_min, double fz_max) const
  {
    // 判断家具类型，与 ROS1 2025 版完全一致
    if (furniture_name.find("table") != std::string::npos)
    {
      if (dz >= fz_min && dz <= fz_max)
      {
        if (dx >= fx_min && dx <= fx_max && dy >= fy_min && dy <= fy_max)
        {
          return "on the " + furniture_name;
        }
      }
      else if (dz < fz_min)
      {
        return "under the " + furniture_name;
      }
    }
    else if (furniture_name.find("trash") != std::string::npos ||
             furniture_name.find("bin")   != std::string::npos)
    {
      if (dz >= fz_min && dz <= fz_max)
      {
        if (dx >= fx_min && dx <= fx_max && dy >= fy_min && dy <= fy_max)
        {
          return "inside the " + furniture_name;
        }
        else
        {
          return "on the " + furniture_name;
        }
      }
    }
    else if (furniture_name.find("cabinet") != std::string::npos ||
             furniture_name.find("shelf")   != std::string::npos)
    {
      if (dz >= fz_min && dz <= fz_max)
      {
        if (dx >= fx_min && dx <= fx_max && dy >= fy_min && dy <= fy_max)
        {
          return "inside the " + furniture_name;
        }
        else
        {
          return "on the " + furniture_name;
        }
      }
    }

    return "near the " + furniture_name;
  }

  std::string getFinalDestination(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr & message)
  {
    taskInfo = *message;

    double dx = taskInfo.destination.position.x;
    double dy = taskInfo.destination.position.y;
    double dz = taskInfo.destination.position.z;

    int   i                        = static_cast<int>(taskInfo.furniture.size());
    int   index_of_nearest_furniture = 0;
    float min_furniture_distance   = 999999999.0f;

    if (i <= 0)
    {
      RCLCPP_WARN(nodeHandle->get_logger(), "getFinalDestination: no furniture, using default");
      return "Place it at the designated destination.";
    }

    for (int j = 0; j < i; ++j)
    {
      float distance =
        std::sqrt(
          std::pow(taskInfo.destination.position.x - taskInfo.furniture[j].position.x, 2) +
          std::pow(std::abs(taskInfo.destination.position.y) - std::abs(taskInfo.furniture[j].position.y), 2) +
          std::pow(taskInfo.destination.position.z - taskInfo.furniture[j].position.z, 2));

      if (distance < min_furniture_distance)
      {
        index_of_nearest_furniture = j;
        min_furniture_distance     = distance;
      }

      double fx    = taskInfo.furniture[index_of_nearest_furniture].position.x;
      double fy    = taskInfo.furniture[index_of_nearest_furniture].position.y;
      double fz    = taskInfo.furniture[index_of_nearest_furniture].position.z;
      double fx_min = fx - std::abs(taskInfo.destination.size.x) / 2.0;
      double fx_max = fx + std::abs(taskInfo.destination.size.x) / 2.0;
      double fy_min = fy - std::abs(taskInfo.destination.size.y) / 2.0;
      double fy_max = fy + std::abs(taskInfo.destination.size.y) / 2.0;
      double fz_min = fz - std::abs(taskInfo.destination.size.z);
      double fz_max = fz + std::abs(taskInfo.destination.size.z);

      if (j == 0)
      {
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "Destination.size: %f, %f, %f",
          taskInfo.destination.size.x,
          std::abs(taskInfo.destination.size.y),
          taskInfo.destination.size.z);
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "dx: %f, dy: %f, dz: %f",
          dx, dy, dz);
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "furniture_amount: %d.",
          i);
      }
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "furniture: %s, index: %d, fx: %f, fy: %f, fz: %f",
        taskInfo.furniture[index_of_nearest_furniture].name.c_str(),
        j, fx, fy, fz);
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Min_Distance: %f",
        min_furniture_distance);

      bool within = (dx <= fx_max && dx >= fx_min && dy <= fy_max && dy >= fy_min);
      if (within)
      {
        RCLCPP_INFO(nodeHandle->get_logger(), "Within the furniture");
        std::string furniture_name = toReadableName(taskInfo.furniture[index_of_nearest_furniture].name);

        std::string relation = getFurnitureRelation(
          furniture_name,
          dx, dy, dz,
          fx_min, fx_max,
          fy_min, fy_max,
          fz_min, fz_max);

        return "Place it " + relation;
      }
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Not within the furniture");
    std::string nearest_furniture_name = toReadableName(taskInfo.furniture[index_of_nearest_furniture].name);

    return "Place it in/on the " + nearest_furniture_name;
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
    if (i <= 0)
    {
      RCLCPP_WARN(nodeHandle->get_logger(), "getInitialDestination: no furniture");
      return "Get " + getHumanReadableTargetName(targetObjectName) + ".";
    }
    for (int j = 0; j < i; ++j)
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

    std::string nearest_furniture = toReadableName(taskInfo.furniture[index_of_nearest_furniture].name);

    int   index_of_nearest_non = 0;
    float min_distance_non     = 999999999.0f;

    int k = static_cast<int>(taskInfo.non_target_objects.size());
    for (int j = 0; j < k; ++j)
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

    std::string readableTarget = getHumanReadableTargetName(targetObjectName);
    std::string nearest_non_target =
      (k > 0) ? toReadableName(taskInfo.non_target_objects[index_of_nearest_non].name) : "";

    if (k > 0 && min_distance_non < 0.4)
    {
      return "Get " + readableTarget + ", close to the " + nearest_furniture;
    }
    else
    {
      return "Get " + readableTarget + ", close to the " + nearest_furniture;
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

    targetObjectName          = taskInfo.target_object.name;

    // 防止除零错误
    if (taskInfo.target_object.position.x != 0)
    {
      direction_target_object =
        std::atan(taskInfo.target_object.position.y / taskInfo.target_object.position.x);
    }
    else
    {
      direction_target_object = M_PI / 2.0;  // x=0时，默认90度
    }

    if (taskInfo.destination.position.x != 0)
    {
      direction_target_direction =
        std::atan(taskInfo.destination.position.y / taskInfo.destination.position.x);
    }
    else
    {
      direction_target_direction = M_PI / 2.0;  // x=0时，默认90度
    }

    final_location   = getFinalDestination(message);
    initial_location = getInitialDestination(message);

    // 在主循环中 polish（见 run()），避免在回调内嵌套 spin
    pending_polish_ = true;

    isTaskInfoReceived = true;
  }

  // receive avatar status
  void avatarStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviAvatarStatus::ConstSharedPtr message)
  {
    avatarStatus = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe avatar status message:" << std::endl <<
      "objctInLeftHand: " << avatarStatus.object_in_left_hand << std::endl <<
      "objectInRightHand: " << avatarStatus.object_in_right_hand << std::endl <<
      "isTargetObjectInLeftHand: " << std::boolalpha
        << static_cast<bool>(avatarStatus.is_target_object_in_left_hand) << std::endl <<
      "isTargetObjectInRightHand: " << std::boolalpha
        << static_cast<bool>(avatarStatus.is_target_object_in_right_hand) << std::endl);

    isSentGetAvatarStatus = false;

    const bool target_flag_is_set =
      static_cast<bool>(avatarStatus.is_target_object_in_left_hand) ||
      static_cast<bool>(avatarStatus.is_target_object_in_right_hand);
    // 某些 AvatarStatus 回包在重新拾取时目标布尔值会迟一拍，名称作为兜底。
    const bool target_name_is_held =
      (!avatarStatus.object_in_left_hand.empty() &&
       avatarStatus.object_in_left_hand == targetObjectName) ||
      (!avatarStatus.object_in_right_hand.empty() &&
       avatarStatus.object_in_right_hand == targetObjectName);
    const bool has_target_object = target_flag_is_set || target_name_is_held;
    const bool has_any_object =
      !avatarStatus.object_in_left_hand.empty() ||
      !avatarStatus.object_in_right_hand.empty();
    const bool has_wrong_object = has_any_object && !has_target_object;
    const std::string wrong_object = !avatarStatus.object_in_left_hand.empty()
                                   ? avatarStatus.object_in_left_hand
                                   : avatarStatus.object_in_right_hand;

    if (step == GuideForPlacement) {
      // 正确物品脱手后先去抖；判定期间重新拿起会取消本次判定。
      if (target_object_was_held_ && !has_target_object) {
        placement_prompt_locked_ = true;
        placement_seeking_target_ = true;
        placement_check_state_ = PlacementCheckState::DropDelay;
        placement_check_deadline_sec_ = nodeHandle->now().seconds() + drop_check_delay_sec_;
        wrong_object_hold_active_ = false;
        last_wrong_object_name_.clear();
      }

      if (has_target_object && placement_prompt_locked_) {
        // 重新拿起：取消延迟、ObjectStatus 等待或成功宽限。
        guidance_queue_.clear();
        placement_check_state_ = PlacementCheckState::Idle;
        placement_prompt_locked_ = false;
        placement_seeking_target_ = false;
        wrong_object_hold_active_ = false;
        last_wrong_object_name_.clear();
        queueGuidance(buildCurrentGuidanceWithDistance(), GuidancePriority::Transition);
      } else if (has_wrong_object &&
                 (!wrong_object_hold_active_ || wrong_object != last_wrong_object_name_)) {
        // 锁定抓错提示；同一物品放下后再抓也会形成新的上升沿。
        guidance_queue_.clear();
        queueGuidance(
          "This is not the object. Please try again!\n" +
          buildDirectionAndDistance(false),
          GuidancePriority::Error);
        placement_prompt_locked_ = true;
        wrong_object_hold_active_ = true;
        last_wrong_object_name_ = wrong_object;
      } else if (!has_any_object && wrong_object_hold_active_) {
        // 放下错误物品：立即恢复目标物的最新方位/距离提示。
        guidance_queue_.clear();
        wrong_object_hold_active_ = false;
        last_wrong_object_name_.clear();
        placement_prompt_locked_ = false;
        placement_seeking_target_ = true;
        queueGuidance(buildCurrentGuidanceWithDistance(), GuidancePriority::Transition);
      }

      target_object_was_held_ = has_target_object;
    }

    // X 请求在最新状态回包到达的这一刻生成完整提示，距离不会沿用旧缓存。
    if (awaiting_requested_avatar_status_) {
      if (!placement_prompt_locked_) {
        queueGuidance(buildCurrentGuidanceWithDistance(), GuidancePriority::Requested);
      }
      awaiting_requested_avatar_status_ = false;
    }

    // 锁定提示必须一直显示到下一次拿起物品，不能被自动目的地提示覆盖。
    if (step == GuideForPlacement && placement_prompt_locked_) {
      return;
    }

    // 规则≤15条：每隔 N 秒更新钟点方位、距离和静态参照关系。
    const double now_sec = nodeHandle->now().seconds();
    // 抓取正确后保留字幕3秒，避免被新的周期字幕立刻覆盖
    if (subtitle_hold_until_sec_ > 0.0) {
      if (now_sec < subtitle_hold_until_sec_) {
        return;
      }
      // 保持窗口结束：立即允许下一条周期提示触发
      subtitle_hold_until_sec_ = 0.0;
      last_direction_hint_sec_ = now_sec - direction_hint_interval_sec_;
    }
    const bool interval_elapsed = (now_sec - last_direction_hint_sec_) >= direction_hint_interval_sec_;

    // 抓错物体且未放下时：锁定错误提示，不允许周期提示覆盖
    if (step == GuideForTakingObject && wrong_object_hold_active_) {
      return;
    }

    if (step == GuideForTakingObject && interval_elapsed)
    {
      sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
      guideMsg = buildCurrentGuidanceWithDistance();
      if (queueGuidance(guideMsg, GuidancePriority::Automatic)) {
        last_direction_hint_sec_ = now_sec;
      }
    }
    else if (step == GuideForPlacement && interval_elapsed)
    {
      sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
      // 放下目标物后临时切回目标物方位；重新拿起后才使用目的地方位。
      guideMsg = buildCurrentGuidanceWithDistance();
      if (queueGuidance(guideMsg, GuidancePriority::Automatic)) {
        last_direction_hint_sec_ = now_sec;
      }
    }
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
      // 原 ROS1 打印对象，ROS2 中简单输出名称或使用 to_yaml 也可，这里只输出 name 以保持简洁
      std::cout << taskInfo.non_target_objects[i].name << std::endl;
    }

    isSentGetObjectStatus = false;

    // 只在明确等待本次请求时消费回包，启动前缓存的 ObjectStatus 不参与判定。
    if (placement_check_state_ == PlacementCheckState::WaitingObjectStatus) {
      const auto & position = objectStatus.target_object.position;
      // B 优先于 A，重叠时按完成处理。
      if (isTargetInDestinationRegion(position)) {
        finishWithCompletionGuidance();
      } else if (isTargetInInitialRegion(position)) {
        lockPlacementGuidance(
          "You just picked up the correct object.", GuidancePriority::Transition);
        placement_check_state_ = PlacementCheckState::SuccessGrace;
        placement_check_deadline_sec_ =
          nodeHandle->now().seconds() + placement_success_grace_sec_;
      } else {
        lockPlacementGuidance(
          "You put the object in the wrong place.\n" +
          buildDirectionAndDistance(true),
          GuidancePriority::Error);
        placement_check_state_ = PlacementCheckState::Idle;
      }
    }
  }

  bool queueGuidance(
    const std::string & message,
    GuidancePriority priority,
    const std::string & display_type = "All")
  {
    if (isFinished || message.empty() || guidance_sent_count_ >= MAX_GUIDANCE_COUNT) {
      return false;
    }

    const auto queued_automatic = static_cast<size_t>(std::count_if(
      guidance_queue_.begin(), guidance_queue_.end(),
      [](const PendingGuidance & item) {
        return item.priority == GuidancePriority::Automatic;
      }));
    if (priority == GuidancePriority::Automatic &&
        automatic_guidance_sent_count_ + queued_automatic >= MAX_AUTOMATIC_GUIDANCE_COUNT) {
      RCLCPP_WARN(nodeHandle->get_logger(), "Automatic guidance budget exhausted (12/12)");
      return false;
    }

    // 周期提示只保留最新一条；连续按下请求键也只保留一个待处理请求。
    if (priority == GuidancePriority::Automatic || priority == GuidancePriority::Requested) {
      auto existing = std::find_if(
        guidance_queue_.begin(), guidance_queue_.end(),
        [priority](const PendingGuidance & item) {return item.priority == priority;});
      if (existing != guidance_queue_.end()) {
        existing->message = message;
        existing->display_type = display_type;
        return true;
      }
    }

    const size_t remaining = MAX_GUIDANCE_COUNT - guidance_sent_count_;
    if (guidance_queue_.size() >= remaining) {
      auto lower_priority = std::find_if(
        guidance_queue_.rbegin(), guidance_queue_.rend(),
        [priority](const PendingGuidance & item) {
          return static_cast<int>(item.priority) < static_cast<int>(priority);
        });
      if (lower_priority == guidance_queue_.rend()) {
        RCLCPP_WARN(nodeHandle->get_logger(), "Guidance queue rejected: 15-message limit reserved");
        return false;
      }
      guidance_queue_.erase(std::next(lower_priority).base());
    }

    PendingGuidance pending{message, display_type, priority};
    auto insert_at = std::find_if(
      guidance_queue_.begin(), guidance_queue_.end(),
      [priority](const PendingGuidance & item) {
        return static_cast<int>(item.priority) < static_cast<int>(priority);
      });
    guidance_queue_.insert(insert_at, std::move(pending));
    return true;
  }

  void processGuidanceQueue()
  {
    if (isFinished && !completion_guidance_pending_) {
      guidance_queue_.clear();
      return;
    }
    if (guidance_queue_.empty() || guidance_sent_count_ >= MAX_GUIDANCE_COUNT) {
      return;
    }

    const double now_sec = nodeHandle->now().seconds();
    if (last_guidance_sent_sec_ > 0.0 &&
        now_sec - last_guidance_sent_sec_ < guidance_min_interval_sec_) {
      return;
    }

    if (speechState == SpeechState::Speakable) {
      const PendingGuidance pending = guidance_queue_.front();
      guidance_queue_.pop_front();
      sendGuidanceMessage(pubGuidanceMsg, pending.message, pending.display_type);
      ++guidance_sent_count_;
      if (pending.priority == GuidancePriority::Automatic) {
        ++automatic_guidance_sent_count_;
      }
      last_guidance_sent_sec_ = now_sec;
      last_direction_hint_sec_ = now_sec;
      speechState = SpeechState::Speaking;
      if (completion_guidance_pending_) {
        completion_guidance_pending_ = false;
        guidance_queue_.clear();
      }
      RCLCPP_INFO(
        nodeHandle->get_logger(), "Guidance count: %zu/%zu (automatic %zu/%zu)",
        guidance_sent_count_, MAX_GUIDANCE_COUNT,
        automatic_guidance_sent_count_, MAX_AUTOMATIC_GUIDANCE_COUNT);
      return;
    }

    // 查询后必须等待 Speech_state 回包，不能自行假定 TTS 空闲；回包丢失才超时重试。
    const double since_query_sec = now_sec - timePrevSpeechStateConfirmed.seconds();
    if (speechState == SpeechState::WaitingState && since_query_sec >= speech_state_timeout_sec_) {
      RCLCPP_WARN(nodeHandle->get_logger(), "Speech state request timed out; retrying");
      speechState = SpeechState::None;
    }
    if (speechState != SpeechState::WaitingState &&
        since_query_sec >= speech_state_poll_interval_sec_) {
      sendMessage(pubHumanNaviMsg, MSG_GET_SPEECH_STATE);
      timePrevSpeechStateConfirmed = nodeHandle->now();
      speechState = SpeechState::WaitingState;
    }
  }

  void stopGuidanceDispatch()
  {
    guidance_queue_.clear();
    speechState = SpeechState::None;
    RCLCPP_INFO(
      nodeHandle->get_logger(), "Guidance dispatch stopped at %zu/%zu messages",
      guidance_sent_count_, MAX_GUIDANCE_COUNT);
  }

  std::string buildCurrentGuidance() const
  {
    if (step == GuideForPlacement && !placement_seeking_target_) {
      return polished_place_base_.empty() ? truncateUtf8(buildStrictPlaceTemplate(), 400)
                                          : polished_place_base_;
    }
    return polished_pick_base_.empty() ? truncateUtf8(buildStrictPickTemplate(), 400)
                                       : polished_pick_base_;
  }

  std::string buildCurrentGuidanceWithDistance() const
  {
    const double av_x = avatarStatus.body.position.x;
    const double av_y = avatarStatus.body.position.y;
    const bool placing = step == GuideForPlacement && !placement_seeking_target_;
    const double target_x = placing ? taskInfo.destination.position.x
                                    : taskInfo.target_object.position.x;
    const double target_y = placing ? taskInfo.destination.position.y
                                    : taskInfo.target_object.position.y;
    const double distance = std::hypot(av_x - target_x, av_y - target_y);
    const std::string hint = getDirectionHint(
      av_x, av_y, target_x, target_y, avatarStatus.body.orientation);
    std::ostringstream dist_ss;
    dist_ss << std::fixed << std::setprecision(1) << distance;
    return (placing ? "The destination is " : "The object is ") + hint +
           ", " + dist_ss.str() + " meters away.\n" + buildCurrentGuidance();
  }

  std::string buildDirectionAndDistance(bool to_destination) const
  {
    const double av_x = avatarStatus.body.position.x;
    const double av_y = avatarStatus.body.position.y;
    const double target_x = to_destination ? taskInfo.destination.position.x
                                           : taskInfo.target_object.position.x;
    const double target_y = to_destination ? taskInfo.destination.position.y
                                           : taskInfo.target_object.position.y;
    const double distance = std::hypot(av_x - target_x, av_y - target_y);
    const std::string hint = getDirectionHint(
      av_x, av_y, target_x, target_y, avatarStatus.body.orientation);
    std::ostringstream dist_ss;
    dist_ss << std::fixed << std::setprecision(1) << distance;
    return (to_destination ? "The destination is " : "The correct object is ") +
           hint + ", " + dist_ss.str() + " meters away.";
  }

  void lockPlacementGuidance(const std::string & message, GuidancePriority priority)
  {
    guidance_queue_.clear();
    placement_prompt_locked_ = true;
    placement_seeking_target_ = true;
    subtitle_hold_until_sec_ = 0.0;
    queueGuidance(message, priority);
  }

  void finishWithCompletionGuidance()
  {
    if (isFinished) {
      return;
    }
    placement_check_state_ = PlacementCheckState::Idle;
    placement_prompt_locked_ = true;
    guidance_queue_.clear();
    completion_guidance_pending_ = true;
    if (!queueGuidance("Task completed.", GuidancePriority::Error)) {
      completion_guidance_pending_ = false;
      RCLCPP_WARN(nodeHandle->get_logger(), "Completion guidance blocked by 15-message limit");
    }
    isFinished = true;
  }

  bool isTargetInInitialRegion(const geometry_msgs::msg::Point & position) const
  {
    return std::hypot(
      position.x - taskInfo.target_object.position.x,
      position.y - taskInfo.target_object.position.y) <= initial_region_radius_;
  }

  bool isTargetInDestinationRegion(const geometry_msgs::msg::Point & position) const
  {
    const auto & center = taskInfo.destination.position;
    const auto & q_msg = taskInfo.destination.orientation;
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    if (q.length2() < 1e-12) {
      q.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      q.normalize();
    }

    // 将世界坐标差旋转回 destination 局部坐标系，再与三维半尺寸比较。
    const tf2::Vector3 world_delta(
      position.x - center.x, position.y - center.y, position.z - center.z);
    const tf2::Vector3 local = tf2::Matrix3x3(q.inverse()) * world_delta;
    const double half_x = std::abs(taskInfo.destination.size.x) * 0.5 +
                          destination_region_tolerance_;
    const double half_y = std::abs(taskInfo.destination.size.y) * 0.5 +
                          destination_region_tolerance_;
    const double half_z = std::abs(taskInfo.destination.size.z) * 0.5 +
                          destination_region_tolerance_;
    return std::abs(local.x()) <= half_x &&
           std::abs(local.y()) <= half_y &&
           std::abs(local.z()) <= half_z;
  }

  void processPlacementCheck()
  {
    if (isFinished || placement_check_state_ == PlacementCheckState::Idle) {
      return;
    }
    const double now_sec = nodeHandle->now().seconds();
    if (placement_check_state_ == PlacementCheckState::DropDelay &&
        now_sec >= placement_check_deadline_sec_) {
      placement_check_state_ = PlacementCheckState::WaitingObjectStatus;
      object_status_request_sec_ = now_sec;
      isSentGetObjectStatus = true;
      sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
    } else if (placement_check_state_ == PlacementCheckState::WaitingObjectStatus &&
               now_sec - object_status_request_sec_ >= object_status_timeout_sec_) {
      // 超时只重试，不使用旧缓存，也不把超时当成 C 区域。
      object_status_request_sec_ = now_sec;
      isSentGetObjectStatus = true;
      sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
      RCLCPP_WARN(nodeHandle->get_logger(), "ObjectStatus timed out; retrying");
    } else if (placement_check_state_ == PlacementCheckState::SuccessGrace &&
               now_sec >= placement_check_deadline_sec_) {
      lockPlacementGuidance(
        "You put the object in the wrong place.\n" +
        buildDirectionAndDistance(true),
        GuidancePriority::Error);
      placement_check_state_ = PlacementCheckState::Idle;
    }
  }

public:
  int run(int argc, char ** argv)
  {
    (void)argc;
    (void)argv;

    // 原 ROS1: ros::init + ros::NodeHandle -> ROS2: rclcpp::init 在 main 中，类内部创建 Node
    nodeHandle = std::make_shared<rclcpp::Node>("human_navi_sample");

    // 初始化 tf2 Buffer/Listener（替代 ROS1 的 tf::TransformListener）
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(nodeHandle->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::Rate loopRate(10.0);  // 原 ROS1: ros::Rate loopRate(10);

    init();

    RCLCPP_INFO(nodeHandle->get_logger(), "Human Navi sample start!");

    // ROS2 订阅/发布器创建，对应 ROS1 NodeHandle.subscribe/advertise
    subHumanNaviMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_robot",
      100,
      std::bind(&HumanNavigationSample::messageCallback, this, std::placeholders::_1));
    subTaskInfoMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>(
      "/human_navigation/message/task_info",
      1,
      std::bind(&HumanNavigationSample::taskInfoMessageCallback, this, std::placeholders::_1));
    subAvatarStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>(
      "/human_navigation/message/avatar_status",
      1,
      std::bind(&HumanNavigationSample::avatarStatusMessageCallback, this, std::placeholders::_1));
    subObjectStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>(
      "/human_navigation/message/object_status",
      1,
      std::bind(&HumanNavigationSample::objectStatusMessageCallback, this, std::placeholders::_1));

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

    nodeHandle->declare_parameter("direction_hint_interval_sec", 10.0);
    direction_hint_interval_sec_ = nodeHandle->get_parameter("direction_hint_interval_sec").as_double();
    nodeHandle->declare_parameter("guidance_min_interval_sec", 1.0);
    guidance_min_interval_sec_ = nodeHandle->get_parameter("guidance_min_interval_sec").as_double();
    nodeHandle->declare_parameter("speech_state_poll_interval_sec", 0.5);
    speech_state_poll_interval_sec_ =
      nodeHandle->get_parameter("speech_state_poll_interval_sec").as_double();
    nodeHandle->declare_parameter("speech_state_timeout_sec", 2.0);
    speech_state_timeout_sec_ = nodeHandle->get_parameter("speech_state_timeout_sec").as_double();
    nodeHandle->declare_parameter("drop_check_delay_sec", 1.0);
    drop_check_delay_sec_ = nodeHandle->get_parameter("drop_check_delay_sec").as_double();
    nodeHandle->declare_parameter("object_status_timeout_sec", 1.0);
    object_status_timeout_sec_ = nodeHandle->get_parameter("object_status_timeout_sec").as_double();
    nodeHandle->declare_parameter("placement_success_grace_sec", 2.0);
    placement_success_grace_sec_ =
      nodeHandle->get_parameter("placement_success_grace_sec").as_double();
    nodeHandle->declare_parameter("initial_region_radius", 0.5);
    initial_region_radius_ = nodeHandle->get_parameter("initial_region_radius").as_double();
    nodeHandle->declare_parameter("destination_region_tolerance", 0.1);
    destination_region_tolerance_ =
      nodeHandle->get_parameter("destination_region_tolerance").as_double();
    RCLCPP_INFO(nodeHandle->get_logger(), "LLM path removed; deterministic guidance only.");

    timePrevSpeechStateConfirmed = nodeHandle->now();

    rclcpp::Time time = nodeHandle->now();

    while (rclcpp::ok())
    {
      rclcpp::spin_some(nodeHandle);
      processPlacementCheck();
      processGuidanceQueue();

      if (pending_polish_) {
        const std::string draft_pick  = buildStrictPickTemplate();
        const std::string draft_place = buildStrictPlaceTemplate();
        polished_pick_base_  = truncateUtf8(draft_pick, 400);
        polished_place_base_ = truncateUtf8(draft_place, 400);
        pending_polish_ = false;
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "Polished guidance bases (pick len=%zu, place len=%zu)",
          polished_pick_base_.size(),
          polished_place_base_.size());
      }

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
            // session start 后立即朝目标物品方向转向
            moveBaseJointTrajectory(pub_base_trajectory_, 0.0, 0.0, direction_target_object, 5.0);
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

            const bool has_target_object =
              static_cast<bool>(avatarStatus.is_target_object_in_left_hand) ||
              static_cast<bool>(avatarStatus.is_target_object_in_right_hand);
            const bool has_any_object =
              !avatarStatus.object_in_left_hand.empty() ||
              !avatarStatus.object_in_right_hand.empty();
            const bool has_wrong_object = has_any_object && !has_target_object;

            // 用户放下错误物品后，继续保留错误提示4秒再恢复周期提示
            if (wrong_object_hold_active_ && !has_wrong_object) {
              guidance_queue_.clear();
              wrong_object_hold_active_ = false;
              last_wrong_object_name_.clear();
              subtitle_hold_until_sec_ = 0.0;
              queueGuidance(buildCurrentGuidanceWithDistance(), GuidancePriority::Transition);
              last_direction_hint_sec_ = nodeHandle->now().seconds();
            }

            if (has_target_object)
            {
              guideMsg = "Please Keep holding the Object";
              queueGuidance(guideMsg, GuidancePriority::Transition);
              // 短句保留2秒即可读完，避免过久阻塞目的地提示。
              last_direction_hint_sec_ = 0.0;
              subtitle_hold_until_sec_ = nodeHandle->now().seconds() + 2.0;

              // 抓取正确后立即朝第三阶段目的地方向转向，给受试者即时方向提示
              moveBaseJointTrajectory(
                pub_base_trajectory_,
                0.0, 0.0,
                direction_target_direction,
                5.0);

              step             = GuideForPlacement;
              target_object_was_held_ = true;
              placement_prompt_locked_ = false;
              placement_seeking_target_ = false;
              isRequestReceived = true;
              last_wrong_object_name_.clear();
              wrong_object_hold_active_ = false;
              break;
            }
            else if (has_wrong_object)
            {
              const std::string wrong_object = !avatarStatus.object_in_left_hand.empty()
                                             ? avatarStatus.object_in_left_hand
                                             : avatarStatus.object_in_right_hand;
              wrong_object_hold_active_ = true;
              if (wrong_object != last_wrong_object_name_)
              {
                guideMsg = "This is not the object. Please try again!\n" +
                  buildDirectionAndDistance(false);
                queueGuidance(guideMsg, GuidancePriority::Error);
                // 抓错提示保留4秒，避免被后续提示瞬间覆盖
                subtitle_hold_until_sec_ = nodeHandle->now().seconds() + 4.0;
                last_wrong_object_name_ = wrong_object;
              }
              break;
            }
          }

          if (isRequestReceived)
          {
            // Guidance_request 已在回调中进入高优先级队列，这里只负责转向。
            moveBaseJointTrajectory(pub_base_trajectory_, 0.0, 0.0, direction_target_object, 5.0);
            time = nodeHandle->now();
            isRequestReceived = false;
          }
          break;
        }
        case GuideForPlacement:
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "GuideForPlacement");

          // A: 放置阶段也持续请求 avatar status，保证方位/距离提示可持续更新
          if (nodeHandle->now().seconds() - avatar_timer > 0.100)
          {
            sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
            avatar_timer = nodeHandle->now().seconds();
          }

          if (isRequestReceived)
          {
            const double now_sec = nodeHandle->now().seconds();
            // 抓取正确提示后的3秒保持窗口：不发送任何新提示词，避免覆盖
            if (subtitle_hold_until_sec_ > 0.0 && now_sec < subtitle_hold_until_sec_) {
              break;
            }
            moveBaseJointTrajectory(
              pub_base_trajectory_,
              0.0, 0.0,
              direction_target_direction,
              5.0);

            isRequestReceived = false;
          }

          break;
        }
        case TaskFinished:
        {
          reset();
          break;
        }
      }

      loopRate.sleep();
    }

    return 0;
  }
};

std::string HumanNavigationSample::truncateUtf8(const std::string & s, size_t max_chars)
{
  size_t n = 0;
  size_t i = 0;
  while (i < s.size() && n < max_chars) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    size_t w = 1;
    if (c >= 0xF0U) {
      w = 4;
    } else if (c >= 0xE0U) {
      w = 3;
    } else if (c >= 0xC0U) {
      w = 2;
    } else if (c >= 0x80U) {
      ++i;
      continue;
    }
    if (i + w > s.size()) {
      break;
    }
    i += w;
    ++n;
  }
  return s.substr(0, i);
}

std::string HumanNavigationSample::escapeJsonString(const std::string & s)
{
  std::string o;
  o.reserve(s.size() + 8);
  for (const char ch : s) {
    if (ch == '"' || ch == '\\') {
      o += '\\';
    }
    o += ch;
  }
  return o;
}

std::string HumanNavigationSample::toReadableName(const std::string & name)
{
  std::string s = name;
  // 常见 Unity 后缀，先去掉，避免影响尾部噪声识别（如 C01(Clone)）
  for (const std::string tag : {std::string("(Clone)"), std::string("(clone)")}) {
    size_t pos = std::string::npos;
    while ((pos = s.find(tag)) != std::string::npos) {
      s.erase(pos, tag.size());
    }
  }
  std::replace(s.begin(), s.end(), '_', ' ');

  // 压缩多空格并分词，便于仅清理 Unity 结尾噪声（如 C01 / C 02 / _A）
  std::vector<std::string> tokens;
  {
    std::istringstream iss(s);
    std::string t;
    while (iss >> t) {
      tokens.push_back(t);
    }
  }
  auto stripEdgePunct = [](const std::string & t) {
    size_t b = 0;
    size_t e = t.size();
    while (b < e && !std::isalnum(static_cast<unsigned char>(t[b]))) {
      ++b;
    }
    while (e > b && !std::isalnum(static_cast<unsigned char>(t[e - 1]))) {
      --e;
    }
    return (b < e) ? t.substr(b, e - b) : std::string();
  };
  auto isSingleLetter = [](const std::string & t) {
    return t.size() == 1 && std::isalpha(static_cast<unsigned char>(t[0]));
  };
  auto isAllDigits = [](const std::string & t) {
    if (t.empty()) {
      return false;
    }
    for (char ch : t) {
      if (!std::isdigit(static_cast<unsigned char>(ch))) {
        return false;
      }
    }
    return true;
  };
  auto isLetterDigits = [&](const std::string & t) {
    if (t.size() < 2 || !std::isalpha(static_cast<unsigned char>(t[0]))) {
      return false;
    }
    return isAllDigits(t.substr(1));
  };

  bool changed = true;
  while (changed && !tokens.empty()) {
    changed = false;
    const std::string last = stripEdgePunct(tokens.back());
    if (isSingleLetter(last) || isLetterDigits(last) || isAllDigits(last)) {
      tokens.pop_back();
      changed = true;
      continue;
    }
    // 处理 "C 02" / "c 02" 这种两段后缀
    if (tokens.size() >= 2 &&
      isAllDigits(stripEdgePunct(tokens.back())) &&
      isSingleLetter(stripEdgePunct(tokens[tokens.size() - 2])))
    {
      tokens.pop_back();
      tokens.pop_back();
      changed = true;
      continue;
    }
  }

  std::ostringstream oss;
  for (size_t i = 0; i < tokens.size(); ++i) {
    if (i != 0) {
      oss << ' ';
    }
    oss << tokens[i];
  }
  std::string out = oss.str();
  const std::string out_low = toLowerAscii(out);
  if (out_low == "motatable" || out_low == "mota table" ||
      out_low == "cafe set mota table") {
    return "motatable (round table)";
  }
  if (out_low == "sidetable" || out_low == "side table") {
    return "sidetable (rectangular table)";
  }
  return out;
}

std::string HumanNavigationSample::toLowerAscii(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

bool HumanNavigationSample::isLargeLandmarkName(const std::string & readable_name_lower)
{
  static const std::vector<std::string> kKeywords = {
    "table", "desk", "counter", "cabinet", "cupboard",
    "drawer", "shelf", "wardrobe", "refrigerator", "fridge"};
  for (const auto & kw : kKeywords) {
    if (readable_name_lower.find(kw) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool HumanNavigationSample::isRefrigeratorName(const std::string & readable_name_lower)
{
  return readable_name_lower.find("refrigerator") != std::string::npos ||
         readable_name_lower.find("fridge") != std::string::npos;
}

std::string HumanNavigationSample::extractThePhraseLower(const std::string & phrase)
{
  if (phrase.empty()) {
    return "";
  }
  const std::string low = toLowerAscii(phrase);
  const size_t pos = low.find("the ");
  if (pos != std::string::npos) {
    return low.substr(pos);
  }
  return low;
}

bool HumanNavigationSample::isFusedSlotName(const std::string & readable_name) const
{
  const std::string full = toLowerAscii(readable_name);
  if (full.find(' ') == std::string::npos) {
    return false;
  }

  std::unordered_set<std::string> known;
  known.reserve(taskInfo.furniture.size() + taskInfo.non_target_objects.size() + 8);
  for (const auto & f : taskInfo.furniture) {
    const std::string n = toLowerAscii(toReadableName(f.name));
    if (!n.empty()) {
      known.insert(n);
    }
  }
  for (const auto & o : taskInfo.non_target_objects) {
    const std::string n = toLowerAscii(toReadableName(o.name));
    if (!n.empty()) {
      known.insert(n);
    }
  }

  size_t pos = full.find(' ');
  while (pos != std::string::npos) {
    const std::string left = full.substr(0, pos);
    const std::string right = (pos + 1 < full.size()) ? full.substr(pos + 1) : std::string();
    if (!left.empty() && !right.empty() &&
      known.count(left) > 0 &&
      known.count(right) > 0 &&
      left != full && right != full)
    {
      return true;
    }
    pos = full.find(' ', pos + 1);
  }
  return false;
}

std::string HumanNavigationSample::normalizeKnownFusionPhrase(std::string text) const
{
  auto replaceAll = [](std::string & s, const std::string & from, const std::string & to) {
    size_t pos = s.find(from);
    while (pos != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos = s.find(from, pos + to.size());
    }
  };
  replaceAll(text, "cafe set mota table", "motatable (round table)");
  replaceAll(text, "Cafe Set Mota Table", "motatable (round table)");
  replaceAll(text, "Cafe set mota table", "motatable (round table)");
  replaceAll(text, "cafe_set_mota_table", "motatable (round table)");
  return text;
}

std::string HumanNavigationSample::getRelationPhraseForPosition(
  double dx, double dy, double dz,
  double size_x, double size_y, double size_z) const
{
  if (taskInfo.furniture.empty()) {
    return "";
  }
  // 简化版 on 判定：
  // 1) 仅考虑在目标点下方的家具（支撑物 Z 小于物体 Z，即 fz < dz）；
  // 2) 在下方候选中按水平距离 dxy 最近选支撑物；
  // 3) 若无下方候选，再回退最近家具作为 near 参照。
  std::string best_relation_below;
  double best_below_dxy = 1e12;
  double best_below_dz = 1e12;
  std::string nearest_name;
  double nearest_dist = 1e12;

  for (const auto & f : taskInfo.furniture) {
    const double fx = f.position.x;
    const double fy = f.position.y;
    const double fz = f.position.z;
    const double dxy = std::sqrt(std::pow(dx - fx, 2) + std::pow(dy - fy, 2));
    const double d3 = std::sqrt(std::pow(dx - fx, 2) + std::pow(dy - fy, 2) + std::pow(dz - fz, 2));
    if (d3 < nearest_dist) {
      nearest_dist = d3;
      const std::string nearest_readable = toReadableName(f.name);
      if (!isFusedSlotName(nearest_readable)) {
        nearest_name = nearest_readable;
      }
    }

    const std::string name = toReadableName(f.name);
    if (isFusedSlotName(name)) {
      continue;
    }
    // 场景中“在物体下方”=> 家具 Z 小于物体 Z
    const double dz_rel = dz - fz;
    if (dz_rel <= 0.0) {
      continue;
    }

    // 语句生成沿用现有接口，减少外部行为波动
    const double fx_min = fx - std::abs(size_x) / 2.0;
    const double fx_max = fx + std::abs(size_x) / 2.0;
    const double fy_min = fy - std::abs(size_y) / 2.0;
    const double fy_max = fy + std::abs(size_y) / 2.0;
    const double fz_min = fz - std::abs(size_z);
    const double fz_max = fz + std::abs(size_z);
    const std::string relation = getFurnitureRelation(
      name, dx, dy, dz,
      fx_min, fx_max, fy_min, fy_max, fz_min, fz_max);
    const std::string rel_low = toLowerAscii(relation);
    const bool support =
      rel_low.rfind("on the ", 0) == 0 ||
      rel_low.rfind("in the ", 0) == 0 ||
      rel_low.rfind("inside the ", 0) == 0;
    if (!support) {
      continue;
    }

    if (dxy < best_below_dxy ||
      (std::abs(dxy - best_below_dxy) < 1e-6 && dz_rel < best_below_dz))
    {
      best_below_dxy = dxy;
      best_below_dz = dz_rel;
      best_relation_below = relation;
    }
  }

  if (!best_relation_below.empty()) {
    return best_relation_below;
  }
  if (!nearest_name.empty()) {
    return "near the " + nearest_name;
  }
  return "";
}

std::string HumanNavigationSample::nearestFurnitureName(double x, double y, double z) const
{
  if (taskInfo.furniture.empty()) {
    return "";
  }
  size_t idx = 0;
  double best = 1e12;
  bool found = false;
  for (size_t i = 0; i < taskInfo.furniture.size(); ++i) {
    const auto & f = taskInfo.furniture[i];
    const std::string readable = toReadableName(f.name);
    if (isFusedSlotName(readable)) {
      continue;
    }
    const double d = std::sqrt(
      std::pow(x - f.position.x, 2) +
      std::pow(y - f.position.y, 2) +
      std::pow(z - f.position.z, 2));
    if (d < best) {
      best = d;
      idx = i;
      found = true;
    }
  }
  if (!found) {
    return "";
  }
  return "the " + toReadableName(taskInfo.furniture[idx].name);
}

std::string HumanNavigationSample::buildPickOnSlotPhrase(
  double px, double py, double pz,
  const std::unordered_set<std::string> & banned_lower) const
{
  if (taskInfo.furniture.empty()) {
    return "";
  }

  // 支撑关系优先：抓取前先判断目标物体是否在家具表面/容器内
  const std::string rel = getRelationPhraseForPosition(px, py, pz, 0.3, 0.3, 0.3);
  const std::string rel_low = toLowerAscii(rel);
  const bool is_support_relation =
    rel_low.rfind("on the ", 0) == 0 ||
    rel_low.rfind("in the ", 0) == 0 ||
    rel_low.rfind("inside the ", 0) == 0;
  if (is_support_relation && banned_lower.count(extractThePhraseLower(rel)) == 0) {
    return rel;
  }

  std::string best_within;
  std::string best_any;
  double best_within_d = 1e12;
  double best_any_d = 1e12;
  double best_within_dz = 1e12;
  double best_any_dz = 1e12;

  for (size_t i = 0; i < taskInfo.furniture.size(); ++i) {
    const auto & f = taskInfo.furniture[i];
    const std::string readable = toReadableName(f.name);
    if (isFusedSlotName(readable)) {
      continue;
    }
    const std::string cand = "the " + readable;
    const std::string cand_lower = toLowerAscii(cand);
    if (banned_lower.count(cand_lower) > 0) {
      continue;
    }
    // 支撑物在“下方” => furniture.z < object.z
    const double dz_rel = pz - f.position.z;
    if (dz_rel <= 0.0) {
      continue;
    }
    const double d3 = std::sqrt(
      std::pow(px - f.position.x, 2) +
      std::pow(py - f.position.y, 2) +
      std::pow(pz - f.position.z, 2));
    const double dxy = std::sqrt(
      std::pow(px - f.position.x, 2) +
      std::pow(py - f.position.y, 2));
    // on 搜索初始规则：0-1.5m 内、且在下方的不可交互物品
    if (d3 <= 1.5 &&
      (dxy < best_within_d || (std::abs(dxy - best_within_d) < 1e-6 && dz_rel < best_within_dz)))
    {
      best_within_d = dxy;
      best_within_dz = dz_rel;
      best_within = cand;
    }
    if (dxy < best_any_d || (std::abs(dxy - best_any_d) < 1e-6 && dz_rel < best_any_dz)) {
      best_any_d = dxy;
      best_any_dz = dz_rel;
      best_any = cand;
    }
  }

  if (!best_within.empty()) {
    return "on " + best_within;
  }
  if (!best_any.empty()) {
    return "on " + best_any;
  }
  return "";
}

std::string HumanNavigationSample::buildOnSlotPhrase(
  double px, double py, double pz,
  double sx, double sy, double sz,
  const std::unordered_set<std::string> & banned_lower) const
{
  const std::string rel = getRelationPhraseForPosition(px, py, pz, sx, sy, sz);
  const std::string low = toLowerAscii(rel);
  if ((low.rfind("on the ", 0) == 0 || low.rfind("in the ", 0) == 0) &&
    banned_lower.count(extractThePhraseLower(rel)) == 0)
  {
    return rel;
  }
  if (taskInfo.furniture.empty()) {
    return "";
  }

  std::string best;
  double best_d = 1e12;
  for (const auto & f : taskInfo.furniture) {
    const std::string readable = toReadableName(f.name);
    if (isFusedSlotName(readable)) {
      continue;
    }
    const std::string cand = "the " + readable;
    const std::string cand_lower = toLowerAscii(cand);
    if (banned_lower.count(cand_lower) > 0) {
      continue;
    }
    const double d = std::sqrt(
      std::pow(px - f.position.x, 2) +
      std::pow(py - f.position.y, 2) +
      std::pow(pz - f.position.z, 2));
    if (d < best_d) {
      best_d = d;
      best = cand;
    }
  }
  return best.empty() ? "" : ("on " + best);
}

// near：0-3m 扫描可交互+不可交互；按最近项规则去重/排重后选第一个合格项
std::string HumanNavigationSample::nearestNearLandmarkName(
  double x, double y, double z,
  double min_distance_m,
  double max_distance_m,
  const std::unordered_set<std::string> & banned_lower) const
{
  struct Candidate
  {
    std::string phrase;
    std::string lower;
    double distance;
    bool is_interactive;
  };
  std::vector<Candidate> candidates;

  auto append_candidate = [&](const std::string & raw_name, double ox, double oy, double oz, bool is_interactive) {
    const double d = std::sqrt(
      std::pow(x - ox, 2) + std::pow(y - oy, 2) + std::pow(z - oz, 2));
    if (d < min_distance_m || d > max_distance_m) {
      return;
    }
    const std::string readable = toReadableName(raw_name);
    if (isFusedSlotName(readable)) {
      return;
    }
    const std::string cand = "the " + readable;
    const std::string cand_lower = toLowerAscii(cand);
    candidates.push_back(Candidate{cand, cand_lower, d, is_interactive});
  };

  for (const auto & f : taskInfo.furniture) {
    append_candidate(f.name, f.position.x, f.position.y, f.position.z, false);
  }
  for (const auto & o : taskInfo.non_target_objects) {
    append_candidate(o.name, o.position.x, o.position.y, o.position.z, true);
  }
  append_candidate(
    taskInfo.target_object.name,
    taskInfo.target_object.position.x,
    taskInfo.target_object.position.y,
    taskInfo.target_object.position.z,
    true);

  if (candidates.empty()) {
    return "";
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.distance < b.distance;
  });

  std::unordered_map<std::string, int> interactive_counts;
  for (const auto & c : candidates) {
    if (c.is_interactive) {
      interactive_counts[c.lower] += 1;
    }
  }

  std::unordered_set<std::string> rejected = banned_lower;
  for (const auto & c : candidates) {
    if (rejected.count(c.lower) > 0) {
      continue;
    }
    if (!c.is_interactive) {
      return c.phrase;
    }
    if (interactive_counts[c.lower] > 1) {
      rejected.insert(c.lower);
      continue;
    }
    return c.phrase;
  }
  return "";
}

// 只使用非目标物体作为第二参照，避免把“地标”和“附近物体”混成同一层描述。
// 同名物体出现多次时不使用该名称，防止产生歧义。
std::string HumanNavigationSample::nearestNearbyObjectName(
  double x, double y, double z,
  double max_distance_m,
  const std::unordered_set<std::string> & banned_lower) const
{
  std::unordered_map<std::string, size_t> name_counts;
  for (const auto & object : taskInfo.non_target_objects) {
    const std::string lower = toLowerAscii("the " + toReadableName(object.name));
    ++name_counts[lower];
  }

  std::string nearest;
  double nearest_distance = max_distance_m;
  for (const auto & object : taskInfo.non_target_objects) {
    const std::string readable = toReadableName(object.name);
    const std::string phrase = "the " + readable;
    const std::string lower = toLowerAscii(phrase);
    if (banned_lower.count(lower) > 0 || name_counts[lower] != 1) {
      continue;
    }
    const double distance = std::sqrt(
      std::pow(x - object.position.x, 2) +
      std::pow(y - object.position.y, 2) +
      std::pow(z - object.position.z, 2));
    if (distance <= nearest_distance) {
      nearest_distance = distance;
      nearest = phrase;
    }
  }
  return nearest;
}

std::string HumanNavigationSample::buildStrictPickTemplate() const
{
  const std::string target = "the " + toReadableName(taskInfo.target_object.name);
  const double tx = taskInfo.target_object.position.x;
  const double ty = taskInfo.target_object.position.y;
  const double tz = taskInfo.target_object.position.z;
  std::unordered_set<std::string> on_banned = {toLowerAscii(target)};
  const std::string place_slot = buildOnSlotPhrase(
    taskInfo.destination.position.x, taskInfo.destination.position.y, taskInfo.destination.position.z,
    std::abs(taskInfo.destination.size.x), std::abs(taskInfo.destination.size.y), std::abs(taskInfo.destination.size.z),
    on_banned);
  const std::string place_the = extractThePhraseLower(place_slot);
  if (!place_the.empty()) {
    on_banned.insert(place_the);
  }
  // 抓取前 on：目标物下方大型不可交互物体，先 0-2m，失败再扩圈
  const std::string on_slot = buildPickOnSlotPhrase(tx, ty, tz, on_banned);
  std::unordered_set<std::string> near_banned = on_banned;
  const std::string on_the = extractThePhraseLower(on_slot);
  if (!on_the.empty()) {
    near_banned.insert(on_the);
  }
  const std::string nearby = nearestNearbyObjectName(tx, ty, tz, 3.0, near_banned);

  if (!on_slot.empty() && !nearby.empty()) {
    return normalizeKnownFusionPhrase(
      target + " is " + on_slot + ", near " + nearby + ".");
  }
  if (!on_slot.empty()) {
    return normalizeKnownFusionPhrase(target + " is " + on_slot + ".");
  }
  if (!nearby.empty()) {
    return normalizeKnownFusionPhrase(target + " is near " + nearby + ".");
  }
  return normalizeKnownFusionPhrase("The target is " + target + ".");
}

std::string HumanNavigationSample::buildStrictPlaceTemplate() const
{
  const double dx = taskInfo.destination.position.x;
  const double dy = taskInfo.destination.position.y;
  const double dz = taskInfo.destination.position.z;
  const double sx = std::abs(taskInfo.destination.size.x);
  const double sy = std::abs(taskInfo.destination.size.y);
  const double sz = std::abs(taskInfo.destination.size.z);
  std::unordered_set<std::string> on_banned = {std::string("the ") + toLowerAscii(toReadableName(taskInfo.target_object.name))};
  const std::string on_slot = buildOnSlotPhrase(dx, dy, dz, sx, sy, sz, on_banned);
  if (!on_slot.empty()) {
    return normalizeKnownFusionPhrase("The destination is " + on_slot + ".");
  }
  return "This is the destination location.";
}

std::string HumanNavigationSample::getDirectionHint(
  double av_x, double av_y,
  double target_x, double target_y,
  const geometry_msgs::msg::Quaternion & body_orient) const
{
  const double dx = target_x - av_x;
  const double dy = target_y - av_y;

  tf2::Quaternion tf_q;
  tf2::fromMsg(body_orient, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  const double forward_x = std::cos(yaw);
  const double forward_y = std::sin(yaw);
  const double to_target_angle = std::atan2(dy, dx);
  double rel = to_target_angle - std::atan2(forward_y, forward_x);
  while (rel > M_PI) rel -= 2.0 * M_PI;
  while (rel < -M_PI) rel += 2.0 * M_PI;

  // Avatar 正前方为 12 点，右侧为 3 点，后方为 6 点，左侧为 9 点。
  // rel 在数学坐标系中逆时针为正，因此换算钟点时取负号。
  constexpr double CLOCK_SECTOR = M_PI / 6.0;
  int hour = (12 - static_cast<int>(std::lround(rel / CLOCK_SECTOR))) % 12;
  if (hour <= 0) {
    hour += 12;
  }
  return "at your " + std::to_string(hour) + " o'clock";
}

std::string HumanNavigationSample::buildContextJson() const
{
  const double tx = taskInfo.target_object.position.x;
  const double ty = taskInfo.target_object.position.y;
  const double tz = taskInfo.target_object.position.z;
  const double dx = taskInfo.destination.position.x;
  const double dy = taskInfo.destination.position.y;
  const double dz = taskInfo.destination.position.z;
  const double sx = std::abs(taskInfo.destination.size.x);
  const double sy = std::abs(taskInfo.destination.size.y);
  const double sz = std::abs(taskInfo.destination.size.z);

  std::unordered_set<std::string> pick_on_banned = {std::string("the ") + toLowerAscii(toReadableName(taskInfo.target_object.name))};
  const std::string place_slot = buildOnSlotPhrase(dx, dy, dz, sx, sy, sz, pick_on_banned);
  const std::string place_the = extractThePhraseLower(place_slot);
  if (!place_the.empty()) {
    pick_on_banned.insert(place_the);
  }
  const std::string pick_slot = buildPickOnSlotPhrase(tx, ty, tz, pick_on_banned);
  auto extractThePhrase = [](const std::string & s) -> std::string {
    if (s.empty()) {
      return "";
    }
    const std::string low = HumanNavigationSample::toLowerAscii(s);
    const size_t pos = low.find("the ");
    if (pos != std::string::npos) {
      return s.substr(pos);
    }
    return s;
  };
  std::string pick_on = extractThePhrase(pick_slot);
  std::string place_on = extractThePhrase(place_slot);
  std::unordered_set<std::string> pick_near_banned = {std::string("the ") + toLowerAscii(toReadableName(taskInfo.target_object.name))};
  if (!pick_on.empty()) {
    pick_near_banned.insert(toLowerAscii(pick_on));
  }
  if (!place_on.empty()) {
    pick_near_banned.insert(toLowerAscii(place_on));
  }
  std::unordered_set<std::string> place_near_banned = {std::string("the ") + toLowerAscii(toReadableName(taskInfo.target_object.name))};
  if (!place_on.empty()) {
    place_near_banned.insert(toLowerAscii(place_on));
  }
  std::string pick_near = nearestNearLandmarkName(tx, ty, tz, 0.0, 3.0, pick_near_banned);
  std::string place_near = nearestNearLandmarkName(dx, dy, dz, 0.0, 3.0, place_near_banned);

  std::ostringstream oss;
  oss << "{\"environment_id\":\"" << escapeJsonString(taskInfo.environment_id) << "\""
      << ",\"target_prefab\":\"" << escapeJsonString(toReadableName(taskInfo.target_object.name)) << "\""
      << ",\"pick_on\":\"" << escapeJsonString(pick_on) << "\""
      << ",\"pick_near\":\"" << escapeJsonString(pick_near) << "\""
      << ",\"place_on\":\"" << escapeJsonString(place_on) << "\""
      << ",\"place_near\":\"" << escapeJsonString(place_near) << "\"}";
  return oss.str();
}

std::string HumanNavigationSample::polishGuidance(
  const std::string & draft, const std::string & phase, const std::string & context_json)
{
  (void)phase;
  (void)context_json;
  return truncateUtf8(normalizeKnownFusionPhrase(draft), 400);
}

int main(int argc, char ** argv)
{
  // 原 ROS1: ros::init(argc, argv, "human_navi_sample");
  // ROS2: 使用 rclcpp::init / shutdown
  rclcpp::init(argc, argv);

  HumanNavigationSample humanNaviSample;
  int ret = humanNaviSample.run(argc, argv);

  rclcpp::shutdown();
  return ret;
}
