#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <interactive_cleanup_msgs/msg/interactive_cleanup_msg.hpp>
#include <cleanup_vision_ros2/msg/avatar_observation.hpp>
#include <cleanup_vision_ros2/msg/hand_target_alignment.hpp>
#include <cleanup_vision_ros2/msg/pointing_direction.hpp>
#include <cleanup_vision_ros2/msg/scene_object_array.hpp>
#include "interactive_cleanup/avatar_tracker.hpp"
#include "interactive_cleanup/grasp_utils.hpp"
#include "interactive_cleanup/hand_servo.hpp"
#include "interactive_cleanup/hsr_geometry.hpp"
#include "interactive_cleanup/navigation_utils.hpp"
#include "interactive_cleanup/observation_head_sweep.hpp"
#include "interactive_cleanup/pick_target_resolver.hpp"
#include "interactive_cleanup/place_destination_resolver.hpp"
#include "interactive_cleanup/pointing_alignment.hpp"
#include "interactive_cleanup/pregrasp_planner.hpp"

#include <array>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <limits>
#include <deque>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using AvatarObservation = cleanup_vision_ros2::msg::AvatarObservation;
using HandTargetAlignment = cleanup_vision_ros2::msg::HandTargetAlignment;
using PointingDirection = cleanup_vision_ros2::msg::PointingDirection;
using SceneObjectArray = cleanup_vision_ros2::msg::SceneObjectArray;
using SceneObject = cleanup_vision_ros2::msg::SceneObject;

class InteractiveCleanupSample : public rclcpp::Node
{
public:
  InteractiveCleanupSample()
  : rclcpp::Node("interactive_cleanup_sample")
  {
  }

  int run()
  {
    // ---- Publishers ----
    pub_msg_ = this->create_publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>(
        "/interactive_cleanup/message/to_moderator", 10);
    pub_msg_human_ = this->create_publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>(
        "/interactive_cleanup/message/to_human", 10);
    pub_hsr_msg_ = this->create_publisher<std_msgs::msg::String>(
        "/hsrb/message/to_human", 10);
    pub_base_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/hsrb/command_velocity", 10);
    pub_arm_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/arm_trajectory_controller/command", 10);
    pub_gripper_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/gripper_controller/command", 10);
    pub_head_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/head_trajectory_controller/command", 10);
    pub_perception_mode_ = this->create_publisher<std_msgs::msg::String>(
        "/cleanup_perception/mode", 10);

    // ---- Subscribers ----
    sub_msg_ = this->create_subscription<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>(
        "/interactive_cleanup/message/to_robot", 100,
        std::bind(&InteractiveCleanupSample::messageCallback, this, std::placeholders::_1));
    sub_hsr_msg_ = this->create_subscription<std_msgs::msg::String>(
        "/hsrb/message/to_robot", 100,
        std::bind(&InteractiveCleanupSample::hsrMessageCallback, this, std::placeholders::_1));
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hsrb/joint_states", 10,
        std::bind(&InteractiveCleanupSample::jointStateCallback, this, std::placeholders::_1));
    sub_avatar_ = this->create_subscription<AvatarObservation>(
        "/cleanup_perception/head/avatar", 10,
        std::bind(&InteractiveCleanupSample::avatarCallback, this, std::placeholders::_1));
    sub_detected_objects_ = this->create_subscription<SceneObjectArray>(
        "/cleanup_perception/head/objects", 10,
        std::bind(&InteractiveCleanupSample::objectsCallback, this, std::placeholders::_1));
    sub_pointing_ = this->create_subscription<PointingDirection>(
        "/cleanup_perception/head/pointing", 10,
        std::bind(&InteractiveCleanupSample::pointingCallback, this, std::placeholders::_1));
    sub_hand_alignment_ = this->create_subscription<HandTargetAlignment>(
        "/cleanup_perception/hand/target_alignment", 10,
        std::bind(&InteractiveCleanupSample::handAlignmentCallback, this, std::placeholders::_1));

    // ---- TF ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Nav2 ----
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // ---- Arm trajectory template ----
    arm_joint_trajectory_.joint_names = {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {0.0, 0.0, 0.0, 0.0, 0.0};
    arm_joint_trajectory_.points.push_back(pt);

    this->declare_parameter<std::string>("destination_regions_file", "");
    const auto destination_regions_file =
      this->get_parameter("destination_regions_file").as_string();
    std::string destination_region_error;
    destination_regions_ = interactive_cleanup::loadDestinationRegions(
      destination_regions_file, &destination_region_error);
    if (destination_regions_.empty()) {
      for (const auto &[name, coord] : KNOWN_DESTINATIONS) {
        destination_regions_.push_back(
          interactive_cleanup::inferDestinationRegion(name, coord.first, coord.second));
      }
      RCLCPP_WARN(
        this->get_logger(),
        "Destination region config unavailable ('%s'), using inferred defaults: %s",
        destination_regions_file.c_str(),
        destination_region_error.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Loaded %zu destination regions from '%s'",
        destination_regions_.size(),
        destination_regions_file.c_str());
    }

    step_ = Initialize;

    RCLCPP_INFO(this->get_logger(), "Interactive Cleanup sample started!");

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      runStateMachine();
      loop_rate.sleep();
    }
    return EXIT_SUCCESS;
  }

private:
  // =========================================================================
  // State definitions
  // =========================================================================
  enum Step {
    Initialize,
    Ready,
    WaitForPickTrackAvatar,
    ResolvePickTarget,
    WaitForCleanTrackAvatar,
    ResolvePlaceDestination,
    PlanPregrasp,
    NavigateToPregrasp,
    DeployArmForApproach,
    HandCameraApproach,
    CloseAndVerifyGrasp,
    SendObjectGrasped,
    MoveToDestination,
    ReleaseObject,
    SendTaskFinished,
    WaitForResult
  };

  // =========================================================================
  // Message constants
  // =========================================================================
  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_I_AM_READY       = "I_am_ready";
  const std::string MSG_PICK_IT_UP       = "Pick_it_up!";
  const std::string MSG_CLEAN_UP         = "Clean_up!";
  const std::string MSG_IS_THIS_CORRECT  = "Is_this_correct?";
  const std::string MSG_POINT_IT_AGAIN   = "Point_it_again";
  const std::string MSG_YES              = "Yes";
  const std::string MSG_NO               = "No";
  const std::string MSG_OBJECT_GRASPED   = "Object_grasped";
  const std::string MSG_TASK_FINISHED    = "Task_finished";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";
  const std::string MSG_GIVE_UP          = "Give_up";

  // =========================================================================
  // Arm poses: {arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll}
  // =========================================================================
  const std::vector<double> ARM_INITIAL       = {0.0,   0.0,   0.0,  0.0,   0.0};
  const std::vector<double> ARM_OBSERVE       = {0.1,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_GRASP_READY   = {0.1,  -0.5,   0.0, -1.0,   0.0};
  const std::vector<double> ARM_GRASP_LOW     = {0.05, -0.8,   0.0, -0.7,   0.0};
  const std::vector<double> ARM_CARRY         = {0.3,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_RELEASE_READY = {0.2,  -0.8,   0.0, -1.0,   0.0};
  const std::vector<double> ARM_RELEASE_LOW   = {0.05, -1.0,   0.0, -0.5,   0.0};

  // =========================================================================
  // Timeouts
  // =========================================================================
  static constexpr double TIMEOUT_WAIT_COMMAND = 120.0;
  static constexpr double TIMEOUT_OBSERVE      =   9.0;
  static constexpr double TIMEOUT_MOVE         =  60.0;
  static constexpr double TIMEOUT_GRASP        =  30.0;
  static constexpr double TIMEOUT_RELEASE      =  30.0;
  static constexpr double TIMEOUT_WAIT_RESULT  =  60.0;
  static constexpr double READY_ACK_RESEND_INTERVAL = 0.5;
  static constexpr double POINTING_MAX_AGE_SEC =   1.5;
  static constexpr double CUE_CAPTURE_WINDOW_SEC = 3.0;
  static constexpr double POINT_ALIGN_ANGLE_THRESHOLD = 0.18;
  static constexpr double POINT_ALIGN_MAX_ANGULAR     = 0.45;
  static constexpr double POINT_ALIGN_STABILITY_THRESHOLD = 0.20;
  static constexpr double POINT_ALIGN_SAMPLE_WINDOW_SEC = 0.8;
  static constexpr double POINT_ALIGN_TIMEOUT_MIN = 2.0;
  static constexpr double POINT_ALIGN_TIMEOUT_MAX = 8.0;
  static constexpr double POINT_ALIGN_TIMEOUT_OVERHEAD = 1.0;
  static constexpr int    POINT_ALIGN_MIN_SAMPLES = 3;
  static constexpr int    POINT_ALIGN_REQUIRED_STABLE_CYCLES = 3;
  static constexpr int    MAX_POINT_ALIGN_RETRY = 0;
  static constexpr int    MAX_POINT_AGAIN      =   2;
  static constexpr double PICK_HEAD_SWEEP_TILT = -0.20;
  static constexpr double PICK_HEAD_SWEEP_PAN = 0.18;
  static constexpr double PICK_HEAD_SWEEP_MOTION = 0.35;
  static constexpr double PICK_HEAD_SWEEP_OBSERVE = 0.40;
  static constexpr double PICK_HEAD_SWEEP_TIMEOUT_OVERHEAD = 0.25;

  static constexpr double APPROACH_STOP_DIST   = 0.45;
  static constexpr double NAV_APPROACH_STANDOFF = 0.75;
  static constexpr double TURN_THRESHOLD       = 0.15;
  static constexpr double VISUAL_SERVO_FWD     = 0.08;
  static constexpr double VISUAL_SERVO_MAX_ANGULAR = 0.35;
  static constexpr double FINAL_APPROACH_FALLBACK_SPEED = 0.06;
  static constexpr double FINAL_APPROACH_FALLBACK_ANGULAR = 0.25;
  static constexpr double FINAL_APPROACH_MIN_TIMEOUT = 12.0;
  static constexpr double FINAL_APPROACH_MAX_TIMEOUT = 30.0;
  static constexpr double FINAL_APPROACH_TIMEOUT_OVERHEAD = 2.0;
  static constexpr double FINAL_APPROACH_OBJECT_MAX_AGE = 1.0;
  static constexpr double FINAL_APPROACH_CLOSE_ENOUGH_TOLERANCE = 0.08;
  static constexpr double FINAL_APPROACH_TIMEOUT_GRASP_TOLERANCE = 0.12;
  static constexpr double FINAL_APPROACH_PROGRESS_DELTA = 0.03;
  static constexpr double FINAL_APPROACH_STALL_TIMEOUT = 4.0;
  static constexpr double FINAL_APPROACH_MIN_PROGRESS_SPEED = 0.02;
  static constexpr int    MAX_GRASP_RETRIES = 2;
  static constexpr double PRE_GRASP_TARGET_FORWARD = 0.40;
  static constexpr double PRE_GRASP_TARGET_LATERAL = 0.0;
  static constexpr double PRE_GRASP_FORWARD_TOLERANCE = 0.03;
  static constexpr double PRE_GRASP_LATERAL_TOLERANCE = 0.02;
  static constexpr double PRE_GRASP_MAX_LINEAR_X = 0.05;
  static constexpr double PRE_GRASP_MAX_LINEAR_Y = 0.04;
  static constexpr double PRE_GRASP_ALIGN_TIMEOUT = 3.0;
  static constexpr double HAND_SERVO_MAX_LIFT_DELTA = 0.03;
  static constexpr double HAND_SERVO_LIFT_COMMAND_INTERVAL = 0.25;
  static constexpr double HAND_SERVO_MIN_LIFT_DELTA = 0.003;
  static constexpr double GRIPPER_CLOSED_POSITION = -0.105;
  static constexpr double GRIPPER_HOLDING_MARGIN = 0.02;
  static constexpr double TARGET_PERSISTENCE_RADIUS = 0.18;

  // =========================================================================
  // Known destination coordinates (from environment config)
  // =========================================================================
  const std::map<std::string, std::pair<double, double>> KNOWN_DESTINATIONS = {
      {"trash_box_for_recycle",    {-4.620, -5.170}},
      {"trash_box_for_burnable",   {-2.830, -5.170}},
      {"trash_box_for_bottle_can", {-1.000, -5.170}},
      {"square_low_table",         {-5.397, -3.129}},
      {"wagon_1",                  {-5.026, -1.117}},
      {"wagon_2",                  {-5.380,  0.814}},
      {"white_side_table_1",       { 1.110, -2.280}},
      {"white_side_table_2",       {-0.740,  1.416}},
      {"cardboard_box",            { 1.098, -0.324}},
      {"wooden_shelf",             {-3.094,  1.530}},
  };

  // =========================================================================
  // State variables
  // =========================================================================
  int step_;
  bool is_started_  = false;
  bool is_finished_ = false;
  bool is_failed_   = false;
  std::string perception_mode_{"IDLE"};
  std::string failed_detail_;
  bool received_pick_command_  = false;
  bool received_clean_command_ = false;
  bool received_yes_ = false;
  bool received_no_  = false;

  std::chrono::steady_clock::time_point state_enter_time_;
  bool state_timer_initialized_ = false;
  int sub_step_ = 0;
  int point_again_count_ = 0;
  int point_align_retry_count_ = 0;
  bool ready_ack_sent_ = false;
  std::chrono::steady_clock::time_point last_ready_ack_time_;
  std::chrono::steady_clock::time_point last_avatar_recv_time_;
  std::chrono::steady_clock::time_point last_pointing_recv_time_;
  std::chrono::steady_clock::time_point last_objects_recv_time_;
  std::chrono::steady_clock::time_point last_hand_alignment_recv_time_;
  std::chrono::steady_clock::time_point observe_align_start_time_;
  bool observe_align_active_ = false;
  double observe_align_timeout_sec_ = POINT_ALIGN_TIMEOUT_MIN;
  bool observe_align_has_initial_error_ = false;
  int observe_align_stable_cycles_ = 0;
  int hand_target_missing_frames_ = 0;
  bool collect_observation_objects_ = false;
  bool collect_observation_pointings_ = false;
  interactive_cleanup::ObservationHeadSweepPlan current_pick_head_sweep_;
  std::size_t current_pick_head_sweep_phase_ = std::numeric_limits<std::size_t>::max();

  // Joint state
  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;

  // Nav2
  bool nav_goal_sent_    = false;
  bool nav_goal_reached_ = false;
  bool nav_goal_failed_  = false;
  bool nav2_available_   = false;

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  // ---- Perception state ----
  AvatarObservation   latest_avatar_;
  SceneObjectArray    latest_objects_;
  PointingDirection   latest_pointing_;
  HandTargetAlignment latest_hand_alignment_;
  bool has_new_avatar_ = false;
  bool has_new_objects_ = false;
  bool has_new_pointing_ = false;
  bool has_new_hand_alignment_ = false;

  struct TimedPointingSample
  {
    PointingDirection msg;
    std::chrono::steady_clock::time_point received_time;
  };

  struct TimedSceneObjectSample
  {
    SceneObjectArray msg;
    std::chrono::steady_clock::time_point received_time;
  };

  // Accumulated observations for voting
  std::vector<SceneObjectArray> obs_objects_;
  std::vector<TimedPointingSample> obs_pointings_;
  std::vector<TimedPointingSample> observe_align_seed_pointings_;
  std::deque<TimedSceneObjectSample> recent_cue_objects_;
  std::deque<TimedPointingSample> recent_cue_pointings_;
  std::vector<SceneObjectArray> latched_pick_objects_;
  std::vector<TimedPointingSample> latched_pick_pointings_;
  std::vector<SceneObjectArray> latched_place_objects_;
  std::vector<TimedPointingSample> latched_place_pointings_;
  bool pick_cue_latched_ = false;
  bool place_cue_latched_ = false;

  // Selected targets
  geometry_msgs::msg::Point target_position_;
  bool   target_valid_ = false;
  std::string target_class_;
  std::string target_grasp_mode_;

  geometry_msgs::msg::Point dest_position_;
  bool   dest_valid_ = false;
  std::string dest_class_;
  std::vector<interactive_cleanup::DestinationRegion> destination_regions_;
  int grasp_retry_count_ = 0;
  bool grasp_verified_ = false;
  interactive_cleanup::PregraspPlan current_pregrasp_plan_;
  interactive_cleanup::HsrArmPose current_hand_approach_pose_;
  std::chrono::steady_clock::time_point last_hand_lift_command_time_;

  // =========================================================================
  // ROS handles
  // =========================================================================
  rclcpp::Publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>::SharedPtr pub_msg_;
  rclcpp::Publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>::SharedPtr pub_msg_human_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hsr_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_trajectory_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_perception_mode_;

  rclcpp::Subscription<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>::SharedPtr sub_msg_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hsr_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<AvatarObservation>::SharedPtr sub_avatar_;
  rclcpp::Subscription<SceneObjectArray>::SharedPtr sub_detected_objects_;
  rclcpp::Subscription<PointingDirection>::SharedPtr   sub_pointing_;
  rclcpp::Subscription<HandTargetAlignment>::SharedPtr sub_hand_alignment_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // =========================================================================
  // Callbacks
  // =========================================================================
  void messageCallback(const interactive_cleanup_msgs::msg::InteractiveCleanupMsg::SharedPtr msg)
  {
    handleIncomingMessage(msg->message, msg->detail, "interactive_cleanup");
  }

  void hsrMessageCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    handleIncomingMessage(msg->data, "", "hsrb");
  }

  void handleIncomingMessage(
    const std::string &message,
    const std::string &detail,
    const char *source)
  {
    RCLCPP_INFO(
      this->get_logger(), "Recv[%s]: '%s' detail='%s'",
      source, message.c_str(), detail.c_str());

    if (message == MSG_ARE_YOU_READY) {
      if (step_ == Ready) {
        is_started_ = true;
        sendReadyAck("initial handshake");
      } else if (step_ == WaitForPickTrackAvatar) {
        sendReadyAck("moderator retried handshake");
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring unexpected Are_you_ready? while in [%s]",
          stepName(step_).c_str());
      }
    }
    else if (message == MSG_PICK_IT_UP) {
      if (step_ == WaitForPickTrackAvatar) {
        received_pick_command_ = true;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring unexpected Pick_it_up! while in [%s]",
          stepName(step_).c_str());
      }
    }
    else if (message == MSG_CLEAN_UP) {
      if (step_ == WaitForCleanTrackAvatar) {
        received_clean_command_ = true;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring unexpected Clean_up! while in [%s]",
          stepName(step_).c_str());
      }
    }
    else if (message == MSG_YES)            { received_yes_ = true; }
    else if (message == MSG_NO)             { received_no_ = true; }
    else if (message == MSG_TASK_SUCCEEDED) { is_finished_ = true; }
    else if (message == MSG_TASK_FAILED) {
      is_failed_ = true;
      failed_detail_ = detail;
      RCLCPP_WARN(this->get_logger(), "Task failed: '%s'", detail.c_str());
    }
    else if (message == MSG_MISSION_COMPLETE) {
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      emergencyStop();
      rclcpp::shutdown();
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(joint_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_positions_[msg->name[i]] = msg->position[i];
  }

  void avatarCallback(const AvatarObservation::SharedPtr msg)
  {
    latest_avatar_ = *msg;
    has_new_avatar_ = true;
    last_avatar_recv_time_ = std::chrono::steady_clock::now();
  }

  void objectsCallback(const SceneObjectArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    has_new_objects_ = true;
    const auto recv_time = std::chrono::steady_clock::now();
    last_objects_recv_time_ = recv_time;
    if (step_ == WaitForPickTrackAvatar || step_ == WaitForCleanTrackAvatar) {
      recent_cue_objects_.push_back(TimedSceneObjectSample{*msg, recv_time});
      pruneCueCaptureWindow();
    }
    if ((step_ == ResolvePickTarget || step_ == ResolvePlaceDestination) &&
        collect_observation_objects_)
      obs_objects_.push_back(*msg);
  }

  void pointingCallback(const PointingDirection::SharedPtr msg)
  {
    latest_pointing_ = *msg;
    has_new_pointing_ = true;
    const auto recv_time = std::chrono::steady_clock::now();
    last_pointing_recv_time_ = recv_time;
    if (step_ == WaitForPickTrackAvatar || step_ == WaitForCleanTrackAvatar) {
      recent_cue_pointings_.push_back(TimedPointingSample{*msg, recv_time});
      pruneCueCaptureWindow();
    }
    if ((step_ == ResolvePickTarget || step_ == ResolvePlaceDestination) &&
        collect_observation_pointings_)
      obs_pointings_.push_back(TimedPointingSample{*msg, recv_time});
  }

  void handAlignmentCallback(const HandTargetAlignment::SharedPtr msg)
  {
    latest_hand_alignment_ = *msg;
    has_new_hand_alignment_ = true;
    last_hand_alignment_recv_time_ = std::chrono::steady_clock::now();
  }

  // =========================================================================
  // Joint helpers
  // =========================================================================
  double getJointPosition(const std::string &name, double def = 0.0)
  {
    std::lock_guard<std::mutex> lk(joint_mutex_);
    auto it = joint_positions_.find(name);
    return (it != joint_positions_.end()) ? it->second : def;
  }

  // =========================================================================
  // Message helpers
  // =========================================================================
  void sendMessage(const std::string &message, const std::string &detail = "")
  {
    RCLCPP_INFO(this->get_logger(), "Send: '%s'", message.c_str());
    interactive_cleanup_msgs::msg::InteractiveCleanupMsg m;
    m.message = message;
    m.detail = detail;
    pub_msg_->publish(m);
    pub_msg_human_->publish(m);

    std_msgs::msg::String hsr_msg;
    hsr_msg.data = message;
    pub_hsr_msg_->publish(hsr_msg);
  }

  void sendReadyAck(const char *reason)
  {
    auto now_time = std::chrono::steady_clock::now();
    if (ready_ack_sent_ &&
        std::chrono::duration<double>(now_time - last_ready_ack_time_).count() <
          READY_ACK_RESEND_INTERVAL) {
      return;
    }

    if (reason != nullptr) {
      RCLCPP_INFO(this->get_logger(), "Ack Are_you_ready?: %s", reason);
    }
    sendMessage(MSG_I_AM_READY);
    ready_ack_sent_ = true;
    last_ready_ack_time_ = now_time;
  }

  // =========================================================================
  // Base motion
  // =========================================================================
  void moveBase(double lx, double ly, double az)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = lx; t.linear.y = ly; t.angular.z = az;
    pub_base_twist_->publish(t);
  }
  void stopBase()            { moveBase(0, 0, 0); }
  void turnBase(double az)   { moveBase(0, 0, az); }
  void driveForward(double v){ moveBase(v, 0, 0); }

  // =========================================================================
  // Arm control
  // =========================================================================
  void moveArm(const std::vector<double> &pos, double dur)
  {
    arm_joint_trajectory_.points[0].positions = pos;
    arm_joint_trajectory_.points[0].time_from_start =
        rclcpp::Duration::from_seconds(dur);
    pub_arm_trajectory_->publish(arm_joint_trajectory_);
  }
  void armInitial()      { moveArm(ARM_INITIAL, 2.0); }
  void armObserve()      { moveArm(ARM_OBSERVE, 1.5); }
  void armGraspReady()   { moveArm(ARM_GRASP_READY, 1.5); }
  void armGraspLow()     { moveArm(ARM_GRASP_LOW, 1.5); }
  void armCarry()        { moveArm(ARM_CARRY, 1.5); }
  void armReleaseReady() { moveArm(ARM_RELEASE_READY, 1.5); }
  void armReleaseLow()   { moveArm(ARM_RELEASE_LOW, 1.5); }

  void applyHandServoLiftDelta(double lift_delta, double duration = 0.35)
  {
    if (std::abs(lift_delta) < HAND_SERVO_MIN_LIFT_DELTA) {
      return;
    }

    const auto now_time = std::chrono::steady_clock::now();
    if (last_hand_lift_command_time_ != std::chrono::steady_clock::time_point{}) {
      const double since_last = std::chrono::duration<double>(
        now_time - last_hand_lift_command_time_).count();
      if (since_last < HAND_SERVO_LIFT_COMMAND_INTERVAL) {
        return;
      }
    }

    interactive_cleanup::HsrArmPose updated_pose = current_hand_approach_pose_;
    updated_pose.arm_lift += lift_delta;
    updated_pose = interactive_cleanup::clampArmPoseToLimits(updated_pose);

    if (std::abs(updated_pose.arm_lift - current_hand_approach_pose_.arm_lift) <
        HAND_SERVO_MIN_LIFT_DELTA) {
      return;
    }

    current_hand_approach_pose_ = updated_pose;
    last_hand_lift_command_time_ = now_time;
    moveArm(toVector(current_hand_approach_pose_), duration);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[HandCameraApproach] Arm lift servo delta=%.3f target=%.3f",
      lift_delta,
      current_hand_approach_pose_.arm_lift);
  }

  // =========================================================================
  // Head control
  // =========================================================================
  void moveHead(double pan, double tilt, double dur)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"head_pan_joint", "head_tilt_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {pan, tilt};
    p.time_from_start = rclcpp::Duration::from_seconds(dur);
    jt.points.push_back(p);
    pub_head_trajectory_->publish(jt);
  }
  void headCenter()    { moveHead(0.0,  0.0, 1.0); }
  void headLookDown()  { moveHead(0.0, -0.5, 1.0); }

  // =========================================================================
  // Gripper
  // =========================================================================
  void operateHand(bool grasp)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"hand_motor_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {grasp ? GRIPPER_CLOSED_POSITION : 1.239};
    p.time_from_start = rclcpp::Duration::from_seconds(2.0);
    jt.points.push_back(p);
    pub_gripper_trajectory_->publish(jt);
  }
  void gripperOpen()  { operateHand(false); }
  void gripperClose() { operateHand(true); }

  // =========================================================================
  // Safety
  // =========================================================================
  void emergencyStop()
  {
    stopBase();
    armInitial();
    headCenter();
    setPerceptionMode("IDLE", true);
    RCLCPP_WARN(this->get_logger(), "Emergency stop");
  }

  void giveUp()
  {
    RCLCPP_WARN(this->get_logger(), "Giving up");
    emergencyStop();
    sendMessage(MSG_GIVE_UP);
  }

  // =========================================================================
  // Vision helpers
  // =========================================================================
  void setPerceptionMode(const std::string &mode, bool force = false)
  {
    if (!force && perception_mode_ == mode) {
      return;
    }
    perception_mode_ = mode;
    std_msgs::msg::String msg;
    msg.data = mode;
    pub_perception_mode_->publish(msg);
  }

  void pruneCueCaptureWindow()
  {
    const auto now_time = std::chrono::steady_clock::now();

    while (!recent_cue_objects_.empty()) {
      const double age = std::chrono::duration<double>(
        now_time - recent_cue_objects_.front().received_time).count();
      if (age <= CUE_CAPTURE_WINDOW_SEC) {
        break;
      }
      recent_cue_objects_.pop_front();
    }

    while (!recent_cue_pointings_.empty()) {
      const double age = std::chrono::duration<double>(
        now_time - recent_cue_pointings_.front().received_time).count();
      if (age <= CUE_CAPTURE_WINDOW_SEC) {
        break;
      }
      recent_cue_pointings_.pop_front();
    }
  }

  void resetCueCaptureWindow()
  {
    recent_cue_objects_.clear();
    recent_cue_pointings_.clear();
  }

  std::vector<SceneObjectArray> snapshotCueObjects() const
  {
    std::vector<SceneObjectArray> frames;
    frames.reserve(recent_cue_objects_.size());
    for (const auto &sample : recent_cue_objects_) {
      frames.push_back(sample.msg);
    }
    return frames;
  }

  std::vector<TimedPointingSample> snapshotCuePointings() const
  {
    std::vector<TimedPointingSample> pointings;
    pointings.reserve(recent_cue_pointings_.size());
    for (const auto &sample : recent_cue_pointings_) {
      pointings.push_back(sample);
    }
    return pointings;
  }

  void clearLatchedPickCue()
  {
    pick_cue_latched_ = false;
    latched_pick_objects_.clear();
    latched_pick_pointings_.clear();
  }

  void clearLatchedPlaceCue()
  {
    place_cue_latched_ = false;
    latched_place_objects_.clear();
    latched_place_pointings_.clear();
  }

  void latchPickCueFromRecentObservations()
  {
    pruneCueCaptureWindow();
    latched_pick_objects_ = snapshotCueObjects();
    latched_pick_pointings_ = snapshotCuePointings();
    pick_cue_latched_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "[WaitForPickTrackAvatar] Latched pick cue frames=%zu pointings=%zu window=%.1fs",
      latched_pick_objects_.size(),
      latched_pick_pointings_.size(),
      CUE_CAPTURE_WINDOW_SEC);
  }

  void latchPlaceCueFromRecentObservations()
  {
    pruneCueCaptureWindow();
    latched_place_objects_ = snapshotCueObjects();
    latched_place_pointings_ = snapshotCuePointings();
    place_cue_latched_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "[WaitForCleanTrackAvatar] Latched place cue frames=%zu pointings=%zu window=%.1fs",
      latched_place_objects_.size(),
      latched_place_pointings_.size(),
      CUE_CAPTURE_WINDOW_SEC);
  }

  /**
   * Compute perpendicular distance from point (px,py) to a ray
   * starting at (ox,oy) with direction (dx,dy).
   * Returns large value if behind the ray origin.
   */
  static double pointRayDist2D(double px, double py,
                               double ox, double oy,
                               double dx, double dy)
  {
    double len = std::hypot(dx, dy);
    if (len < 1e-6) return 9999.0;
    dx /= len; dy /= len;
    double vx = px - ox, vy = py - oy;
    double proj = vx * dx + vy * dy;
    if (proj < 0) return 9999.0;
    double perp_x = vx - proj * dx;
    double perp_y = vy - proj * dy;
    return std::hypot(perp_x, perp_y);
  }

  /**
   * Compute perpendicular distance from point (px,py,pz) to a 3D ray.
   * Returns large value if behind the ray origin or if the direction is invalid.
   */
  static double pointRayDist3D(double px, double py, double pz,
                               double ox, double oy, double oz,
                               double dx, double dy, double dz,
                               double *along = nullptr)
  {
    double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6) return 9999.0;

    dx /= len; dy /= len; dz /= len;
    double vx = px - ox, vy = py - oy, vz = pz - oz;
    double proj = vx * dx + vy * dy + vz * dz;
    if (along != nullptr) { *along = proj; }
    if (proj < 0.0) return 9999.0;

    double perp_x = vx - proj * dx;
    double perp_y = vy - proj * dy;
    double perp_z = vz - proj * dz;
    return std::sqrt(perp_x * perp_x + perp_y * perp_y + perp_z * perp_z);
  }

  std::vector<interactive_cleanup::PointingObservation> collectPointingObservations(
    const std::vector<TimedPointingSample> &samples) const
  {
    std::vector<interactive_cleanup::PointingObservation> pointings;
    pointings.reserve(samples.size());
    for (const auto &pointing : samples) {
      interactive_cleanup::PointingObservation sample;
      sample.is_valid = pointing.msg.is_valid;
      sample.confidence = pointing.msg.confidence;
      sample.origin = pointing.msg.origin;
      sample.direction = pointing.msg.direction;
      sample.wrist_pixel_x = pointing.msg.wrist_pixel_x;
      sample.wrist_pixel_y = pointing.msg.wrist_pixel_y;
      sample.point_pixel_x = pointing.msg.point_pixel_x;
      sample.point_pixel_y = pointing.msg.point_pixel_y;
      pointings.push_back(sample);
    }
    return pointings;
  }

  bool resolvePickFromObservations(
    const std::vector<SceneObjectArray> &object_frames,
    const std::vector<TimedPointingSample> &pointings)
  {
    std::vector<interactive_cleanup::PickCandidate> candidates;
    for (const auto &frame : object_frames) {
      for (const auto &object : frame.objects) {
        if (object.class_name == "person" || !object.has_3d_position) {
          continue;
        }

        interactive_cleanup::PickCandidate candidate;
        candidate.class_name = object.class_name;
        candidate.confidence = object.confidence;
        candidate.bbox_cx = object.bbox_cx;
        candidate.bbox_cy = object.bbox_cy;
        candidate.bbox_w = object.bbox_w;
        candidate.bbox_h = object.bbox_h;
        candidate.position = object.position;
        candidate.has_3d_position = object.has_3d_position;
        candidates.push_back(candidate);
      }
    }

    const auto result = interactive_cleanup::resolvePickTarget(
      candidates, collectPointingObservations(pointings));
    if (!result.valid) {
      return false;
    }

    target_valid_ = true;
    target_class_ = result.class_name;
    target_position_ = result.position;
    target_grasp_mode_ = result.grasp_mode;

    RCLCPP_INFO(
      this->get_logger(),
      "Selected TARGET: class='%s' pos=(%.2f, %.2f, %.2f) mode=%s score=%.3f",
      target_class_.c_str(),
      target_position_.x,
      target_position_.y,
      target_position_.z,
      target_grasp_mode_.c_str(),
      result.score);
    return true;
  }

  bool resolveDestinationFromObservations(
    const std::vector<TimedPointingSample> &pointings)
  {
    const auto result = interactive_cleanup::resolvePlaceDestination(
      destination_regions_, collectPointingObservations(pointings), target_class_);
    if (!result.valid) {
      return false;
    }

    dest_valid_ = true;
    dest_class_ = result.name;
    dest_position_ = result.position;

    RCLCPP_INFO(
      this->get_logger(),
      "Selected DESTINATION: name='%s' pos=(%.2f, %.2f, %.2f) mode=%s score=%.3f",
      dest_class_.c_str(),
      dest_position_.x,
      dest_position_.y,
      dest_position_.z,
      result.placement_mode.c_str(),
      result.score);
    return true;
  }

  void logObservationSummary(
    const char *label,
    const std::vector<SceneObjectArray> &object_frames) const
  {
    std::map<std::string, int> counts;
    int total_objects = 0;
    int total_objects_3d = 0;

    for (const auto &frame : object_frames) {
      for (const auto &obj : frame.objects) {
        if (obj.class_name == "person") continue;
        total_objects++;
        if (obj.has_3d_position) {
          total_objects_3d++;
        }
        counts[obj.class_name]++;
      }
    }

    if (counts.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] Observation summary: no non-person detections in %zu frames",
                  label, object_frames.size());
      return;
    }

    std::vector<std::pair<std::string, int>> ranked(counts.begin(), counts.end());
    std::sort(
      ranked.begin(), ranked.end(),
      [](const auto &a, const auto &b) {
        if (a.second != b.second) return a.second > b.second;
        return a.first < b.first;
      });

    std::ostringstream oss;
    const size_t limit = std::min<size_t>(ranked.size(), 8);
    for (size_t i = 0; i < limit; ++i) {
      if (i != 0) oss << ", ";
      oss << ranked[i].first << ":" << ranked[i].second;
    }

    RCLCPP_INFO(this->get_logger(),
                "[%s] Observation summary: frames=%zu objs=%d objs3d=%d classes=[%s]",
                label, object_frames.size(), total_objects, total_objects_3d,
                oss.str().c_str());
  }

  // =========================================================================
  // Robot pose helpers (odom frame)
  // =========================================================================
  struct RobotPose { double x, y, yaw; bool valid; };

  RobotPose getRobotPose(const std::string &frame = "odom")
  {
    RobotPose rp{0, 0, 0, false};
    try {
      auto tf = tf_buffer_->lookupTransform(frame, "base_footprint",
                                            tf2::TimePointZero);
      rp.x = tf.transform.translation.x;
      rp.y = tf.transform.translation.y;
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double r, p, ya;
      tf2::Matrix3x3(q).getRPY(r, p, ya);
      rp.yaw = ya;
      rp.valid = true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF error: %s", ex.what());
    }
    return rp;
  }

  double angleToPoint(double tx, double ty, const std::string &frame = "odom")
  {
    auto rp = getRobotPose(frame);
    if (!rp.valid) return 0.0;
    return interactive_cleanup::bearingToTarget({rp.x, rp.y, rp.yaw}, tx, ty);
  }

  double distToPoint(double tx, double ty, const std::string &frame = "odom")
  {
    auto rp = getRobotPose(frame);
    if (!rp.valid) return 0.0;
    return std::hypot(tx - rp.x, ty - rp.y);
  }

  bool buildMapGoalFromOdomTarget(
    const geometry_msgs::msg::Point &odom_target,
    double standoff,
    interactive_cleanup::PlanarGoal &map_goal)
  {
    auto robot_odom = getRobotPose("odom");
    if (!robot_odom.valid) {
      RCLCPP_WARN(this->get_logger(),
                  "Robot pose in odom unavailable while preparing object goal");
      return false;
    }

    try {
      auto map_from_odom = tf_buffer_->lookupTransform(
        "map", "odom", tf2::TimePointZero, tf2::durationFromSec(1.0));
      const auto odom_goal = interactive_cleanup::buildApproachGoal(
        {robot_odom.x, robot_odom.y, robot_odom.yaw},
        odom_target, standoff);
      geometry_msgs::msg::Point odom_goal_point;
      odom_goal_point.x = odom_goal.x;
      odom_goal_point.y = odom_goal.y;
      odom_goal_point.z = 0.0;
      map_goal = interactive_cleanup::transformGoalToMap(
        odom_goal_point, odom_goal.yaw, map_from_odom);
      RCLCPP_INFO(
        this->get_logger(),
        "[MoveToObj] Odom target=(%.2f, %.2f) approach=(%.2f, %.2f, yaw=%.2f) standoff=%.2f",
        odom_target.x, odom_target.y,
        odom_goal.x, odom_goal.y, odom_goal.yaw,
        standoff);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform object goal odom->map: %s", ex.what());
      return false;
    }
  }

  bool hasRecentAvatarObservation(double max_age_sec = 1.0) const
  {
    if (!has_new_avatar_ || !latest_avatar_.is_valid) {
      return false;
    }

    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_avatar_recv_time_).count();
    return age <= max_age_sec;
  }

  void runAvatarTracking(const char *label)
  {
    if (!hasRecentAvatarObservation()) {
      stopBase();
      return;
    }
    const auto cmd = interactive_cleanup::computeAvatarTrackCommand(
      latest_avatar_.center_error_x,
      0.05,
      1.2,
      0.18);
    if (!cmd.tracking) {
      stopBase();
      return;
    }

    turnBase(cmd.angular_z);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[%s] Tracking avatar center_error=%.3f cmd=%.2f",
      label,
      latest_avatar_.center_error_x,
      cmd.angular_z);
  }

  static std::vector<double> toVector(const std::array<double, 5> &pose)
  {
    return std::vector<double>(pose.begin(), pose.end());
  }

  static std::vector<double> toVector(const interactive_cleanup::HsrArmPose &pose)
  {
    return {
      pose.arm_lift,
      pose.arm_flex,
      pose.arm_roll,
      pose.wrist_flex,
      pose.wrist_roll,
    };
  }

  const SceneObject *findBestTargetMatch(
    double max_distance = std::numeric_limits<double>::infinity()) const
  {
    const SceneObject *best = nullptr;
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto &obj : latest_objects_.objects) {
      if (obj.class_name == "person" || !obj.has_3d_position) {
        continue;
      }

      const double spatial_distance = target_valid_
        ? std::hypot(
            obj.position.x - target_position_.x,
            obj.position.y - target_position_.y)
        : std::hypot(obj.bbox_cx - 320.0, obj.bbox_cy - 240.0) / 320.0;

      if (target_valid_ && spatial_distance > max_distance) {
        continue;
      }

      double score = spatial_distance;
      if (!target_class_.empty() && obj.class_name != target_class_) {
        score += 0.5;
      }

      if (score < best_score) {
        best = &obj;
        best_score = score;
      }
    }

    return best;
  }

  bool refreshTargetFromLatestDetections(
    double max_distance = std::numeric_limits<double>::infinity())
  {
    if (!has_new_objects_) {
      return false;
    }

    has_new_objects_ = false;
    const auto *match = findBestTargetMatch(max_distance);
    if (match == nullptr) {
      return false;
    }

    target_position_ = match->position;
    target_class_ = match->class_name;
    target_valid_ = true;
    return true;
  }

  bool transformPointFromOdomToFrame(
    const geometry_msgs::msg::Point &odom_point,
    const std::string &target_frame,
    geometry_msgs::msg::Point &transformed_point)
  {
    try {
      const auto target_from_odom = tf_buffer_->lookupTransform(
        target_frame, "odom", tf2::TimePointZero, tf2::durationFromSec(0.2));
      geometry_msgs::msg::PointStamped odom_point_stamped;
      geometry_msgs::msg::PointStamped target_point_stamped;
      odom_point_stamped.header.frame_id = "odom";
      odom_point_stamped.point = odom_point;
      tf2::doTransform(odom_point_stamped, target_point_stamped, target_from_odom);
      transformed_point = target_point_stamped.point;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform target from odom to %s: %s",
                  target_frame.c_str(), ex.what());
      return false;
    }
  }

  bool buildPregraspPlanFromTarget()
  {
    if (!target_valid_) {
      return false;
    }

    geometry_msgs::msg::Point target_in_base;
    if (!transformPointFromOdomToFrame(target_position_, "base_footprint", target_in_base)) {
      return false;
    }

    current_pregrasp_plan_ = interactive_cleanup::planPregrasp(
      target_in_base, target_grasp_mode_);
    if (!current_pregrasp_plan_.valid) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[PlanPregrasp] target_in_base=(%.2f, %.2f, %.2f) mode=%s standoff=%.2f profile=%s",
      target_in_base.x, target_in_base.y, target_in_base.z,
      current_pregrasp_plan_.grasp_mode.c_str(),
      current_pregrasp_plan_.nav_standoff,
      current_pregrasp_plan_.approach_profile.c_str());
    return true;
  }

  bool hasFreshTargetObservationNearPickup(bool &target_visible)
  {
    target_visible = false;
    if (!has_new_objects_) {
      return false;
    }

    const auto *match = findBestTargetMatch(TARGET_PERSISTENCE_RADIUS);
    target_visible = (match != nullptr);
    return true;
  }

  bool hasFreshHandAlignment(double max_age_sec = 0.8) const
  {
    if (!has_new_hand_alignment_) {
      return false;
    }

    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_hand_alignment_recv_time_).count();
    return age <= max_age_sec;
  }

  bool verifyCurrentGrasp()
  {
    const double hand_motor_joint_position = getJointPosition(
      "hand_motor_joint", GRIPPER_CLOSED_POSITION);
    const bool gripper_holding_object =
      interactive_cleanup::gripperLikelyHoldingObject(
        hand_motor_joint_position,
        GRIPPER_CLOSED_POSITION,
        GRIPPER_HOLDING_MARGIN);

    bool target_visible_near_pickup = false;
    const bool has_fresh_target_observation =
      hasFreshTargetObservationNearPickup(target_visible_near_pickup);
    const bool hand_camera_confirms_grasp =
      hasFreshHandAlignment() &&
      latest_hand_alignment_.is_target_found &&
      latest_hand_alignment_.target_class == target_class_ &&
      latest_hand_alignment_.bbox_area_ratio >= 0.10;

    grasp_verified_ = has_fresh_target_observation
      ? interactive_cleanup::isGraspVerificationSuccessful(
          gripper_holding_object,
          target_visible_near_pickup,
          hand_camera_confirms_grasp)
      : (gripper_holding_object || hand_camera_confirms_grasp);

    RCLCPP_INFO(
      this->get_logger(),
      "[CloseAndVerifyGrasp] Verification: verified=%s hold=%s target_visible=%s fresh=%s hand=%s motor=%.3f",
      grasp_verified_ ? "yes" : "no",
      gripper_holding_object ? "yes" : "no",
      target_visible_near_pickup ? "yes" : "no",
      has_fresh_target_observation ? "yes" : "no",
      hand_camera_confirms_grasp ? "yes" : "no",
      hand_motor_joint_position);
    return grasp_verified_;
  }

  std::vector<interactive_cleanup::PointingObservation> collectRecentAlignmentPointings() const
  {
    std::vector<interactive_cleanup::PointingObservation> pointings;
    if (!observe_align_active_) {
      return pointings;
    }

    auto append_pointing = [&pointings](const TimedPointingSample &pointing) {
      interactive_cleanup::PointingObservation sample;
      sample.is_valid = pointing.msg.is_valid;
      sample.confidence = pointing.msg.confidence;
      sample.origin = pointing.msg.origin;
      sample.direction = pointing.msg.direction;
      sample.wrist_pixel_x = pointing.msg.wrist_pixel_x;
      sample.wrist_pixel_y = pointing.msg.wrist_pixel_y;
      sample.point_pixel_x = pointing.msg.point_pixel_x;
      sample.point_pixel_y = pointing.msg.point_pixel_y;
      pointings.push_back(sample);
    };

    for (const auto &pointing : observe_align_seed_pointings_) {
      append_pointing(pointing);
    }

    const auto now_time = std::chrono::steady_clock::now();
    for (const auto &pointing : obs_pointings_) {
      if (pointing.received_time < observe_align_start_time_) {
        continue;
      }

      const double age = std::chrono::duration<double>(
        now_time - pointing.received_time).count();
      if (age > POINT_ALIGN_SAMPLE_WINDOW_SEC) {
        continue;
      }

      append_pointing(pointing);
    }
    return pointings;
  }

  std::vector<TimedPointingSample> collectPickResolutionPointings() const
  {
    std::vector<TimedPointingSample> pointings;
    pointings.reserve(latched_pick_pointings_.size() + obs_pointings_.size());

    for (const auto &pointing : latched_pick_pointings_) {
      pointings.push_back(pointing);
    }

    for (const auto &pointing : obs_pointings_) {
      if (pointing.received_time < observe_align_start_time_) {
        continue;
      }
      pointings.push_back(pointing);
    }

    return pointings;
  }

  void beginObservationAlignment(
    const char *label,
    const std::vector<TimedPointingSample> &seed_pointings = {})
  {
    observe_align_active_ = true;
    observe_align_start_time_ = std::chrono::steady_clock::now();
    observe_align_timeout_sec_ = POINT_ALIGN_TIMEOUT_MIN;
    observe_align_has_initial_error_ = false;
    observe_align_stable_cycles_ = 0;
    observe_align_seed_pointings_ = seed_pointings;
    obs_objects_.clear();
    obs_pointings_.clear();
    collect_observation_objects_ = true;
    collect_observation_pointings_ = true;
    stopBase();
    headCenter();
    RCLCPP_INFO(this->get_logger(), "[%s] Starting base alignment from pointing", label);
  }

  double observeAlignElapsed() const
  {
    return observe_align_active_
      ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - observe_align_start_time_).count()
      : 0.0;
  }

  double observeAlignTimeout() const
  {
    return observe_align_timeout_sec_;
  }

  bool runPointingAlignment(const char *label)
  {
    const auto robot_pose = getRobotPose();
    if (!robot_pose.valid) {
      stopBase();
      return false;
    }

    const auto pointings = collectRecentAlignmentPointings();
    const auto result = interactive_cleanup::evaluatePointingAlignment(
      pointings,
      robot_pose.yaw,
      POINT_ALIGN_ANGLE_THRESHOLD,
      POINT_ALIGN_STABILITY_THRESHOLD,
      1.6,
      POINT_ALIGN_MAX_ANGULAR,
      POINT_ALIGN_MIN_SAMPLES);

    if (!result.has_measurement) {
      observe_align_stable_cycles_ = 0;
      stopBase();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] Waiting for fresh pointing after state entry", label);
      return false;
    }

    if (!observe_align_has_initial_error_) {
      observe_align_timeout_sec_ = interactive_cleanup::computePointingAlignmentTimeout(
        result.yaw_error,
        POINT_ALIGN_MAX_ANGULAR,
        POINT_ALIGN_TIMEOUT_MIN,
        POINT_ALIGN_TIMEOUT_OVERHEAD,
        POINT_ALIGN_TIMEOUT_MAX);
      observe_align_has_initial_error_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Pointing alignment timeout set to %.2fs from initial err=%.2f",
        label,
        observe_align_timeout_sec_,
        result.yaw_error);
    }

    if (result.aligned) {
      observe_align_stable_cycles_++;
      stopBase();
      if (observe_align_stable_cycles_ >= POINT_ALIGN_REQUIRED_STABLE_CYCLES) {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Pointing alignment complete yaw=%.2f err=%.2f samples=%zu stability=%.2f",
          label,
          result.desired_yaw,
          result.yaw_error,
          result.sample_count,
          result.angular_stability);
        return true;
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] Pointing near target, confirming stability samples=%zu stability=%.2f stable=%d/%d",
        label,
        result.sample_count,
        result.angular_stability,
        observe_align_stable_cycles_,
        POINT_ALIGN_REQUIRED_STABLE_CYCLES);
      return false;
    }

    observe_align_stable_cycles_ = 0;
    if (result.within_angle_threshold) {
      stopBase();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] Waiting for stable pointing samples=%zu stability=%.2f",
        label,
        result.sample_count,
        result.angular_stability);
      return false;
    }

    turnBase(result.command_angular);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[%s] Aligning to pointing yaw=%.2f err=%.2f cmd=%.2f samples=%zu stability=%.2f",
      label,
      result.desired_yaw,
      result.yaw_error,
      result.command_angular,
      result.sample_count,
      result.angular_stability);
    return false;
  }

  void beginPickHeadSweep(const char *label)
  {
    current_pick_head_sweep_ = interactive_cleanup::buildObservationHeadSweepPlan(
      PICK_HEAD_SWEEP_TILT,
      PICK_HEAD_SWEEP_PAN,
      PICK_HEAD_SWEEP_MOTION,
      PICK_HEAD_SWEEP_OBSERVE);
    current_pick_head_sweep_phase_ = std::numeric_limits<std::size_t>::max();
    collect_observation_pointings_ = false;
    collect_observation_objects_ = false;
    enterTimedState();
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Starting head micro-sweep phases=%zu total=%.2fs",
      label,
      current_pick_head_sweep_.phases.size(),
      current_pick_head_sweep_.total_duration_sec);
  }

  double pickHeadSweepTimeout() const
  {
    return current_pick_head_sweep_.total_duration_sec + PICK_HEAD_SWEEP_TIMEOUT_OVERHEAD;
  }

  bool runPickHeadSweep(const char *label)
  {
    const auto sample = interactive_cleanup::sampleObservationHeadSweep(
      current_pick_head_sweep_, elapsed());
    collect_observation_objects_ = sample.collecting;

    if (!sample.complete && sample.phase_index != current_pick_head_sweep_phase_) {
      current_pick_head_sweep_phase_ = sample.phase_index;
      const auto &phase = current_pick_head_sweep_.phases[sample.phase_index];
      moveHead(phase.pan, phase.tilt, phase.motion_duration_sec);
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Head sweep phase %zu/%zu pan=%.2f tilt=%.2f move=%.2fs observe=%.2fs",
        label,
        sample.phase_index + 1,
        current_pick_head_sweep_.phases.size(),
        phase.pan,
        phase.tilt,
        phase.motion_duration_sec,
        phase.observe_duration_sec);
    }

    if (sample.complete) {
      collect_observation_objects_ = false;
      RCLCPP_INFO(this->get_logger(), "[%s] Head micro-sweep complete", label);
      return true;
    }

    if (sample.collecting) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] Head sweep observing phase=%zu/%zu",
        label,
        sample.phase_index + 1,
        current_pick_head_sweep_.phases.size());
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] Waiting for head settle phase=%zu/%zu",
        label,
        sample.phase_index + 1,
        current_pick_head_sweep_.phases.size());
    }

    return false;
  }

  // =========================================================================
  // Nav2
  // =========================================================================
  void sendNavGoal(double x, double y, double yaw)
  {
    if (nav_goal_sent_) return;
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server (up to 10s)...");

    // Non-blocking wait: check in 1s intervals so Ctrl+C is responsive
    bool server_ready = false;
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
      if (nav_client_->wait_for_action_server(1s)) {
        server_ready = true;
        break;
      }
    }
    if (!server_ready) {
      RCLCPP_WARN(this->get_logger(),
                  "Nav2 action server not available after 10s, falling back to direct drive");
      nav2_available_ = false;
      nav_goal_failed_ = true;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Nav2 action server connected");
    nav2_available_ = true;

    tf2::Quaternion q; q.setRPY(0, 0, yaw); q.normalize();
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
      [this](std::shared_ptr<GoalHandleNav> goal_handle) {
        if (!goal_handle) {
          nav_goal_sent_ = false;
          nav_goal_failed_ = true;
          RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected");
        }
      };
    opts.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
      nav_goal_sent_ = false;
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        nav_goal_reached_ = true;
      } else {
        nav_goal_failed_ = true;
      }
    };

    nav_goal_sent_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
    nav_client_->async_send_goal(goal, opts);
    RCLCPP_INFO(this->get_logger(), "Nav goal: (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
  }

  void resetNavState()
  {
    nav_goal_sent_ = false;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
  }

  // =========================================================================
  // Timing
  // =========================================================================
  void reset()
  {
    is_started_ = false;
    is_finished_ = false;
    is_failed_ = false;
    failed_detail_ = "";
    received_pick_command_ = false;
    received_clean_command_ = false;
    received_yes_ = false;
    received_no_ = false;
    state_timer_initialized_ = false;
    sub_step_ = 0;
    point_again_count_ = 0;
    point_align_retry_count_ = 0;
    ready_ack_sent_ = false;
    perception_mode_ = "IDLE";
    has_new_avatar_ = false;
    has_new_objects_ = false;
    has_new_pointing_ = false;
    has_new_hand_alignment_ = false;
    target_valid_ = false;
    target_grasp_mode_.clear();
    dest_valid_ = false;
    grasp_retry_count_ = 0;
    grasp_verified_ = false;
    current_pregrasp_plan_ = interactive_cleanup::PregraspPlan{};
    current_hand_approach_pose_ = interactive_cleanup::HsrArmPose{};
    hand_target_missing_frames_ = 0;
    collect_observation_objects_ = false;
    collect_observation_pointings_ = false;
    current_pick_head_sweep_ = interactive_cleanup::ObservationHeadSweepPlan{};
    current_pick_head_sweep_phase_ = std::numeric_limits<std::size_t>::max();
    obs_objects_.clear();
    obs_pointings_.clear();
    observe_align_seed_pointings_.clear();
    recent_cue_objects_.clear();
    recent_cue_pointings_.clear();
    clearLatchedPickCue();
    clearLatchedPlaceCue();
    observe_align_active_ = false;
    observe_align_timeout_sec_ = POINT_ALIGN_TIMEOUT_MIN;
    observe_align_has_initial_error_ = false;
    observe_align_stable_cycles_ = 0;
    last_avatar_recv_time_ = std::chrono::steady_clock::time_point{};
    last_pointing_recv_time_ = std::chrono::steady_clock::time_point{};
    last_objects_recv_time_ = std::chrono::steady_clock::time_point{};
    last_hand_alignment_recv_time_ = std::chrono::steady_clock::time_point{};
    last_hand_lift_command_time_ = std::chrono::steady_clock::time_point{};
    resetNavState();
  }

  std::string stepName(int s)
  {
    switch (s) {
      case Initialize:       return "Initialize";
      case Ready:            return "Ready";
      case WaitForPickTrackAvatar:  return "WaitForPickTrackAvatar";
      case ResolvePickTarget: return "ResolvePickTarget";
      case WaitForCleanTrackAvatar: return "WaitForCleanTrackAvatar";
      case ResolvePlaceDestination: return "ResolvePlaceDestination";
      case PlanPregrasp:     return "PlanPregrasp";
      case NavigateToPregrasp: return "NavigateToPregrasp";
      case DeployArmForApproach: return "DeployArmForApproach";
      case HandCameraApproach: return "HandCameraApproach";
      case CloseAndVerifyGrasp: return "CloseAndVerifyGrasp";
      case SendObjectGrasped: return "SendGrasped";
      case MoveToDestination: return "MoveToDest";
      case ReleaseObject:    return "ReleaseObj";
      case SendTaskFinished: return "SendFinished";
      case WaitForResult:    return "WaitResult";
      default:               return "?";
    }
  }

  void enterTimedState()
  {
    state_enter_time_ = std::chrono::steady_clock::now();
    state_timer_initialized_ = true;
  }

  void enterNewState()
  {
    enterTimedState();
    sub_step_ = 0;
  }

  double elapsed()
  {
    return state_timer_initialized_
      ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - state_enter_time_).count()
      : 0.0;
  }

  void changeStep(int next)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] -> [%s]",
                stepName(step_).c_str(), stepName(next).c_str());
    step_ = next;
    enterNewState();
  }

  // =========================================================================
  // State machine
  // =========================================================================
  void runStateMachine()
  {
    // Global failure handler
    if (is_failed_) {
      RCLCPP_WARN(this->get_logger(), "FAILED in [%s] detail='%s'",
                  stepName(step_).c_str(), failed_detail_.c_str());
      emergencyStop();
      step_ = Initialize;
      is_failed_ = false;
      return;
    }

    switch (step_) {

    // -----------------------------------------------------------------
    case Initialize: {
      reset();
      armInitial();
      headCenter();
      gripperOpen();
      setPerceptionMode("IDLE", true);
      RCLCPP_INFO(this->get_logger(), "[Init] Waiting for Are_you_ready?");
      step_ = Ready;
      break;
    }

    // -----------------------------------------------------------------
    case Ready: {
      if (is_started_) {
        armObserve();
        headCenter();
        gripperOpen();
        sendReadyAck("transition to task start");
        setPerceptionMode("TRACK_AVATAR", true);
        changeStep(WaitForPickTrackAvatar);
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForPickTrackAvatar: {
      if (sub_step_ == 0) {
        clearLatchedPickCue();
        clearLatchedPlaceCue();
        resetCueCaptureWindow();
        stopBase();
        headCenter();
        setPerceptionMode("TRACK_AVATAR", true);
        target_valid_ = false;
        target_class_.clear();
        target_grasp_mode_.clear();
        dest_valid_ = false;
        dest_class_.clear();
        sub_step_ = 1;
      }
      runAvatarTracking("WaitForPickTrackAvatar");
      if (received_pick_command_) {
        RCLCPP_INFO(this->get_logger(), "[WaitForPick] Got Pick_it_up! → latching pick cue");
        received_pick_command_ = false;
        latchPickCueFromRecentObservations();
        resetCueCaptureWindow();
        stopBase();
        changeStep(WaitForCleanTrackAvatar);
      } else if (state_timer_initialized_ && elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case ResolvePickTarget: {
      if (sub_step_ == 0) {
        stopBase();
        headCenter();
        setPerceptionMode("IDLE");
        logObservationSummary("ResolvePickTarget", latched_pick_objects_);
        const bool found = pick_cue_latched_ &&
          resolvePickFromObservations(latched_pick_objects_, latched_pick_pointings_);
        if (found) {
          point_again_count_ = 0;
          changeStep(ResolvePlaceDestination);
          break;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "[ResolvePickTarget] Latched pick cue did not resolve a target, starting active re-observation");
        beginObservationAlignment("ResolvePickTarget", latched_pick_pointings_);
        setPerceptionMode("RESOLVE_PICK", true);
        sub_step_ = 1;
        break;
      }

      if (sub_step_ == 1) {
        setPerceptionMode("RESOLVE_PICK");
        if (runPointingAlignment("ResolvePickTarget")) {
          observe_align_active_ = false;
          beginPickHeadSweep("ResolvePickTarget");
          sub_step_ = 2;
          break;
        }

        if (observeAlignElapsed() > observeAlignTimeout()) {
          observe_align_active_ = false;
          stopBase();
          RCLCPP_WARN(
            this->get_logger(),
            "[ResolvePickTarget] Pointing alignment timed out, continuing with bounded head sweep");
          beginPickHeadSweep("ResolvePickTarget");
          sub_step_ = 2;
        }
        break;
      }

      if (sub_step_ == 2) {
        setPerceptionMode("RESOLVE_PICK");
        if (!runPickHeadSweep("ResolvePickTarget")) {
          break;
        }

        stopBase();
        headCenter();
        setPerceptionMode("IDLE");
        logObservationSummary("ResolvePickTarget/ActiveObservation", obs_objects_);
        const bool found = resolvePickFromObservations(
          obs_objects_, collectPickResolutionPointings());
        if (found) {
          point_again_count_ = 0;
          changeStep(ResolvePlaceDestination);
        } else if (point_again_count_ < MAX_POINT_AGAIN) {
          RCLCPP_WARN(
            this->get_logger(),
            "[ResolvePickTarget] Active re-observation did not resolve a target, requesting re-point");
          point_again_count_++;
          sendMessage(MSG_POINT_IT_AGAIN);
          clearLatchedPickCue();
          clearLatchedPlaceCue();
          resetCueCaptureWindow();
          changeStep(WaitForPickTrackAvatar);
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "[ResolvePickTarget] No target after active re-observation and max re-points, giving up");
          giveUp();
          changeStep(WaitForResult);
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForCleanTrackAvatar: {
      if (sub_step_ == 0) {
        clearLatchedPlaceCue();
        resetCueCaptureWindow();
        stopBase();
        headCenter();
        setPerceptionMode("TRACK_AVATAR", true);
        sub_step_ = 1;
      }
      runAvatarTracking("WaitForCleanTrackAvatar");
      if (received_clean_command_) {
        RCLCPP_INFO(this->get_logger(), "[WaitForClean] Got Clean_up! → latching place cue");
        received_clean_command_ = false;
        latchPlaceCueFromRecentObservations();
        resetCueCaptureWindow();
        stopBase();
        changeStep(ResolvePickTarget);
      } else if (elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case ResolvePlaceDestination: {
      stopBase();
      headCenter();
      setPerceptionMode("IDLE");
      logObservationSummary("ResolvePlaceDestination", latched_place_objects_);
      const bool found = place_cue_latched_ &&
        resolveDestinationFromObservations(latched_place_pointings_);
      if (!found && point_again_count_ < MAX_POINT_AGAIN) {
        RCLCPP_WARN(
          this->get_logger(),
          "[ResolvePlaceDestination] Latched place cue did not resolve a destination, requesting re-point");
        point_again_count_++;
        sendMessage(MSG_POINT_IT_AGAIN);
        clearLatchedPlaceCue();
        resetCueCaptureWindow();
        changeStep(WaitForCleanTrackAvatar);
      } else if (!found) {
        RCLCPP_WARN(this->get_logger(), "[ResolvePlaceDestination] No destination after max re-points, giving up");
        giveUp();
        changeStep(WaitForResult);
      } else {
        point_again_count_ = 0;
        changeStep(PlanPregrasp);
      }
      break;
    }

    // -----------------------------------------------------------------
    case PlanPregrasp: {
      stopBase();
      setPerceptionMode("IDLE");
      if (buildPregraspPlanFromTarget()) {
        changeStep(NavigateToPregrasp);
      } else if (elapsed() > 1.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "[PlanPregrasp] Could not prepare pregrasp plan");
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case NavigateToPregrasp: {
      if (elapsed() > TIMEOUT_MOVE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0: {
          armObserve();
          headCenter();
          gripperOpen();
          setPerceptionMode("IDLE");
          RCLCPP_INFO(this->get_logger(), "[NavigateToPregrasp] Preparing approach...");

          if (target_valid_) {
            interactive_cleanup::PlanarGoal map_goal;
            if (!buildMapGoalFromOdomTarget(
                target_position_,
                current_pregrasp_plan_.nav_standoff,
                map_goal)) {
              RCLCPP_ERROR(this->get_logger(),
                           "[NavigateToPregrasp] Unable to prepare map-frame goal for target");
              giveUp();
              changeStep(WaitForResult);
              break;
            }
            sendNavGoal(map_goal.x, map_goal.y, map_goal.yaw);
          }
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          if (nav_goal_reached_) {
            RCLCPP_INFO(this->get_logger(), "[NavigateToPregrasp] Nav2 goal reached");
            stopBase();
            changeStep(DeployArmForApproach);
          } else if (nav_goal_failed_ || !nav_goal_sent_) {
            RCLCPP_ERROR(this->get_logger(),
                         "[NavigateToPregrasp] Nav2 failed before safe approach; aborting task");
            stopBase();
            giveUp();
            changeStep(WaitForResult);
          }
          break;
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case DeployArmForApproach: {
      if (sub_step_ == 0) {
        stopBase();
        headCenter();
        gripperOpen();
        setPerceptionMode("HAND_APPROACH", true);
        current_hand_approach_pose_ = current_pregrasp_plan_.arm_pose;
        last_hand_lift_command_time_ = std::chrono::steady_clock::time_point{};
        moveArm(toVector(current_pregrasp_plan_.arm_pose), 1.5);
        sub_step_ = 1;
        enterTimedState();
      } else if (elapsed() > 1.7) {
        hand_target_missing_frames_ = 0;
        changeStep(HandCameraApproach);
      }
      break;
    }

    // -----------------------------------------------------------------
    case HandCameraApproach: {
      if (elapsed() > TIMEOUT_GRASP && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0: {
          setPerceptionMode("HAND_APPROACH", true);
          hand_target_missing_frames_ = 0;
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          const bool target_found = hasFreshHandAlignment() &&
            latest_hand_alignment_.is_target_found &&
            (target_class_.empty() || latest_hand_alignment_.target_class == target_class_);

          if (target_found) {
            hand_target_missing_frames_ = 0;
            interactive_cleanup::HandServoInput servo_input;
            servo_input.target_found = true;
            servo_input.pixel_error_x = latest_hand_alignment_.pixel_error_x * 320.0;
            servo_input.pixel_error_y = latest_hand_alignment_.pixel_error_y * 240.0;
            servo_input.bbox_area_ratio = latest_hand_alignment_.bbox_area_ratio;
            servo_input.in_grasp_window = latest_hand_alignment_.in_grasp_window;
            servo_input.confidence = latest_hand_alignment_.confidence;
            const auto command = interactive_cleanup::computeHandServoCommand(
              servo_input,
              PRE_GRASP_MAX_LINEAR_X,
              PRE_GRASP_MAX_LINEAR_Y,
              HAND_SERVO_MAX_LIFT_DELTA);

            if (command.should_close || command.aligned) {
              stopBase();
              changeStep(CloseAndVerifyGrasp);
              break;
            }

            applyHandServoLiftDelta(command.lift_delta);
            moveBase(command.linear_x, command.linear_y, 0.0);
            RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 500,
              "[HandCameraApproach] Servo target=%s err=(%.2f, %.2f) area=%.3f cmd=(%.3f, %.3f, lift=%.3f)",
              latest_hand_alignment_.target_class.c_str(),
              servo_input.pixel_error_x,
              servo_input.pixel_error_y,
              servo_input.bbox_area_ratio,
              command.linear_x,
              command.linear_y,
              command.lift_delta);
          } else {
            hand_target_missing_frames_++;
            stopBase();
            if (hasFreshHandAlignment()) {
              RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "[HandCameraApproach] Fresh hand alignment but unusable: found=%s class=%s expected=%s conf=%.2f area=%.3f",
                latest_hand_alignment_.is_target_found ? "yes" : "no",
                latest_hand_alignment_.target_class.c_str(),
                target_class_.c_str(),
                latest_hand_alignment_.confidence,
                latest_hand_alignment_.bbox_area_ratio);
            } else {
              RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "[HandCameraApproach] No fresh hand alignment; hand camera may not see the target yet");
            }
          }

          if (interactive_cleanup::shouldRetryHandApproach(
              false,
              target_found,
              hand_target_missing_frames_,
              6,
              elapsed(),
              8.0)) {
            if (grasp_retry_count_ < MAX_GRASP_RETRIES) {
              grasp_retry_count_++;
              sub_step_ = 2;
              enterTimedState();
            } else {
              RCLCPP_ERROR(this->get_logger(),
                           "[HandCameraApproach] Alignment failed after %d retries",
                           MAX_GRASP_RETRIES);
              giveUp();
              changeStep(WaitForResult);
            }
          }
          break;
        }
        case 2:
          if (elapsed() < 0.8) {
            moveBase(-0.04, 0.0, 0.0);
          } else {
            stopBase();
            changeStep(PlanPregrasp);
          }
          break;
      }
      break;
    }

    // -----------------------------------------------------------------
    case CloseAndVerifyGrasp: {
      if (elapsed() > TIMEOUT_GRASP && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0:
          stopBase();
          setPerceptionMode("HAND_VERIFY", true);
          gripperClose();
          sub_step_ = 1;
          enterTimedState();
          break;
        case 1:
          if (elapsed() > 2.0) {
            if (verifyCurrentGrasp()) {
              armCarry();
              sub_step_ = 2;
              enterTimedState();
            } else if (grasp_retry_count_ < MAX_GRASP_RETRIES) {
              grasp_retry_count_++;
              gripperOpen();
              sub_step_ = 3;
              enterTimedState();
            } else {
              RCLCPP_ERROR(this->get_logger(),
                           "[CloseAndVerifyGrasp] Verification failed after %d retries",
                           MAX_GRASP_RETRIES);
              giveUp();
              changeStep(WaitForResult);
            }
          }
          break;
        case 2:
          if (elapsed() > 1.8) {
            setPerceptionMode("IDLE");
            if (grasp_verified_) {
              changeStep(SendObjectGrasped);
            } else {
              giveUp();
              changeStep(WaitForResult);
            }
          }
          break;
        case 3:
          if (elapsed() < 0.8) {
            moveBase(-0.04, 0.0, 0.0);
          } else {
            stopBase();
            changeStep(PlanPregrasp);
          }
          break;
      }
      break;
    }

    // -----------------------------------------------------------------
    case SendObjectGrasped: {
      sendMessage(MSG_OBJECT_GRASPED);
      changeStep(MoveToDestination);
      break;
    }

    // -----------------------------------------------------------------
    case MoveToDestination: {
      if (elapsed() > TIMEOUT_MOVE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0: {
          headCenter();
          RCLCPP_INFO(this->get_logger(), "[MoveToDest] Heading to destination...");

          if (dest_valid_) {
            auto robot_map = getRobotPose("map");
            if (!robot_map.valid) {
              RCLCPP_ERROR(this->get_logger(),
                           "[MoveToDest] Robot pose in map unavailable");
              giveUp();
              changeStep(WaitForResult);
              break;
            }
            double yaw = interactive_cleanup::bearingToTarget(
              {robot_map.x, robot_map.y, robot_map.yaw},
              dest_position_.x, dest_position_.y);
            sendNavGoal(dest_position_.x, dest_position_.y, yaw);
          }
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          if (nav_goal_reached_) {
            stopBase();
            sub_step_ = 3;
            enterTimedState();
          } else if (nav_goal_failed_ || !nav_goal_sent_) {
            RCLCPP_ERROR(this->get_logger(),
                         "[MoveToDest] Nav2 failed before destination approach; aborting task");
            stopBase();
            giveUp();
            changeStep(WaitForResult);
          }
          break;
        }
        case 2: {
          // Direct drive
          if (!dest_valid_) {
            if (elapsed() < 2.0)     { turnBase(0.5); }
            else if (elapsed() < 5.0){ driveForward(0.15); }
            else { stopBase(); sub_step_ = 3; }
            break;
          }

          double dist = distToPoint(dest_position_.x, dest_position_.y);
          double angle = angleToPoint(dest_position_.x, dest_position_.y);

          if (dist > APPROACH_STOP_DIST) {
            if (std::abs(angle) > TURN_THRESHOLD) {
              moveBase(0.05, 0.0, std::clamp(angle * 1.5, -0.5, 0.5));
            } else {
              moveBase(std::min(0.2, dist * 0.4), 0.0, angle * 0.3);
            }
          } else {
            stopBase();
            sub_step_ = 3;
            enterTimedState();
          }
          break;
        }
        case 3: {
          changeStep(ReleaseObject);
          break;
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case ReleaseObject: {
      if (elapsed() > TIMEOUT_RELEASE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0:
          armReleaseReady(); headLookDown();
          sub_step_ = 1; enterTimedState(); break;
        case 1:
          if (elapsed() > 2.0) { armReleaseLow(); sub_step_ = 2; enterTimedState(); }
          break;
        case 2:
          if (elapsed() > 2.0) { gripperOpen(); sub_step_ = 3; enterTimedState(); }
          break;
        case 3:
          if (elapsed() > 3.0) {
            armInitial(); headCenter();
            sub_step_ = 4; enterTimedState();
          }
          break;
        case 4:
          if (elapsed() < 2.0) { driveForward(-0.1); }
          else { stopBase(); changeStep(SendTaskFinished); }
          break;
      }
      break;
    }

    // -----------------------------------------------------------------
    case SendTaskFinished: {
      sendMessage(MSG_TASK_FINISHED);
      changeStep(WaitForResult);
      break;
    }

    // -----------------------------------------------------------------
    case WaitForResult: {
      setPerceptionMode("IDLE");
      if (is_finished_) {
        RCLCPP_INFO(this->get_logger(), "[WaitResult] Task succeeded!");
        step_ = Initialize;
      } else if (elapsed() > TIMEOUT_WAIT_RESULT) {
        RCLCPP_WARN(this->get_logger(), "[WaitResult] Timeout, resetting");
        step_ = Initialize;
      }
      break;
    }

    } // switch
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractiveCleanupSample>();

  // Ensure Ctrl+C triggers clean shutdown
  rclcpp::on_shutdown([node]() {
    RCLCPP_INFO(node->get_logger(), "Shutdown requested, stopping...");
  });

  int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
