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
#include <cleanup_vision_ros2/msg/detected_object_array.hpp>
#include <cleanup_vision_ros2/msg/pointing_direction.hpp>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using DetectedObjectArray = cleanup_vision_ros2::msg::DetectedObjectArray;
using DetectedObject = cleanup_vision_ros2::msg::DetectedObject;
using PointingDirection = cleanup_vision_ros2::msg::PointingDirection;

class InteractiveCleanupSample : public rclcpp::Node
{
public:
  InteractiveCleanupSample()
  : rclcpp::Node("interactive_cleanup_sample")
  {
    nav2_enabled_ = this->declare_parameter<bool>("use_nav2", true);
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
    pub_vision_enable_ = this->create_publisher<std_msgs::msg::Bool>(
        "/cleanup_vision/enable", 10);

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
    sub_detected_objects_ = this->create_subscription<DetectedObjectArray>(
        "/cleanup_vision/detected_objects", 10,
        std::bind(&InteractiveCleanupSample::objectsCallback, this, std::placeholders::_1));
    sub_pointing_ = this->create_subscription<PointingDirection>(
        "/cleanup_vision/pointing_direction", 10,
        std::bind(&InteractiveCleanupSample::pointingCallback, this, std::placeholders::_1));

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
    WaitForPickCommand,
    ObserveTarget,
    WaitForCleanCommand,
    ObserveDestination,
    MoveToObject,
    GraspObject,
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
  // The HSR teleoperation model defines (-1.57, 0.0) as horizontal and
  // (-2.2, 0.35) as downward.  A floor grasp must use the latter; lowering
  // only arm_lift while retaining the horizontal pose leaves the fingers far
  // above an object on the ground.
  const std::vector<double> ARM_GRASP_READY   = {0.10, -2.20,  0.0,  0.35,  0.0};
  const std::vector<double> ARM_GRASP_LOW     = {0.00, -2.20,  0.0,  0.35,  0.0};
  const std::vector<double> ARM_CARRY         = {0.3,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_RELEASE_READY = {0.2,  -0.8,   0.0, -1.0,   0.0};

  // =========================================================================
  // Head scan patterns
  // =========================================================================
  struct HeadPose { double pan; double tilt; };
  const std::vector<HeadPose> SCAN_WIDE = {
      { 0.0, -0.3}, { 0.6, -0.2}, {-0.6, -0.2},
      { 0.0, -0.5}, { 0.8, -0.3}, {-0.8, -0.3}, { 0.0,  0.0},
  };
  const std::vector<HeadPose> SCAN_FORWARD = {
      { 0.0, -0.3}, { 0.0, -0.5}, { 0.2, -0.4}, {-0.2, -0.4}, { 0.0, 0.0},
  };
  const std::vector<HeadPose> SCAN_LOCAL = {
      { 0.0,  -0.25},
      { 0.45, -0.25}, {-0.45, -0.25},
      { 0.85, -0.25}, {-0.85, -0.25},
      { 0.0,  -0.50},
      { 0.45, -0.55}, {-0.45, -0.55},
      { 0.85, -0.60}, {-0.85, -0.60},
      { 0.0,  -0.70},
  };

  // =========================================================================
  // Timeouts
  // =========================================================================
  static constexpr double TIMEOUT_WAIT_COMMAND = 120.0;
  static constexpr double TIMEOUT_OBSERVE      =   9.0;
  static constexpr double TIMEOUT_POINTING_WAIT = 15.0;
  static constexpr double TIMEOUT_MOVE         =  60.0;
  static constexpr double TIMEOUT_GRASP        =  30.0;
  static constexpr double TIMEOUT_RELEASE      =  45.0;
  static constexpr double TIMEOUT_WAIT_RESULT  =  60.0;
  static constexpr double READY_ACK_RESEND_INTERVAL = 0.5;
  static constexpr double POINTING_MAX_AGE_SEC =   1.5;
  static constexpr double POINTING_MEMORY_SEC  =  10.0;
  static constexpr double POINT_ALIGN_TIMEOUT  =   2.0;
  static constexpr double POINT_ALIGN_ANGLE_THRESHOLD = 0.18;
  static constexpr double POINT_REALIGN_THRESHOLD     = 0.28;
  static constexpr double POINT_ALIGN_MAX_ANGULAR     = 0.45;
  static constexpr double AVATAR_TRACK_INTERVAL       = 0.25;
  static constexpr int    MAX_POINT_ALIGN_RETRY = 0;
  static constexpr int    MAX_POINT_AGAIN      =   2;

  // HSR prefab geometry: 0.349 m main arm + 0.1405 m palm, plus fingers.
  // Stop the base outside the object and put the object at the gripper reach,
  // rather than driving the base centre to the object coordinate.
  // First stop far enough away for the head camera to re-localize a floor
  // object.  The final approach is performed later against the frozen target
  // with the real gripper collision centre, so observation and grasp ranges
  // must not share one threshold.
  static constexpr double OBJECT_OBSERVE_STOP_DIST = 0.80;
  static constexpr double OBJECT_OBSERVE_MIN_DIST  = 0.72;
  static constexpr double OBJECT_OBSERVE_MAX_DIST  = 0.90;
  static constexpr double APPROACH_STOP_DIST   = 0.60;
  static constexpr double APPROACH_MIN_SAFE_DIST = 0.55;
  static constexpr double GRIPPER_ALIGN_TIMEOUT = 20.0;
  static constexpr double GRIPPER_ALIGN_FORWARD_TOLERANCE = 0.018;
  static constexpr double GRIPPER_ALIGN_LATERAL_TOLERANCE = 0.012;
  static constexpr int    GRIPPER_ALIGN_STABLE_CYCLES = 3;
  static constexpr double GRIPPER_MAX_HEIGHT_ABOVE_TARGET = 0.08;
  // Destination motion has two distinct stages.  The base first stops where
  // carried-object collisions are unlikely, then the real finger collision
  // midpoint is servoed onto a point inside Unity's placement trigger.
  static constexpr double DESTINATION_COARSE_STOP_DIST = 0.78;
  static constexpr double DESTINATION_MIN_BASE_CLEARANCE = 0.48;
  static constexpr double RELEASE_ALIGN_TIMEOUT = 20.0;
  static constexpr double RELEASE_ALIGN_FORWARD_TOLERANCE = 0.030;
  static constexpr double RELEASE_ALIGN_LATERAL_TOLERANCE = 0.025;
  static constexpr int    RELEASE_ALIGN_STABLE_CYCLES = 3;
  static constexpr double RELEASE_HEIGHT_TOLERANCE = 0.060;
  static constexpr double TURN_THRESHOLD       = 0.15;
  static constexpr double VISUAL_SERVO_TIMEOUT = 12.0;
  static constexpr double VISUAL_SERVO_CENTER_TOLERANCE = 0.10;
  static constexpr double TARGET_EARLY_CONFIRM_DELAY = 1.2;
  static constexpr int    STABLE_TARGET_MIN_FRAMES = 6;
  static constexpr double STABLE_TARGET_MIN_CONFIDENCE = 0.45;
  static constexpr double STABLE_TARGET_MAX_RAY_2D_PX = 100.0;
  // Allow for the simulated avatar's fingertip/pose-ray error while keeping
  // the target well inside the wider 1.5 m per-sample rejection radius.
  static constexpr double STABLE_TARGET_MAX_RAY_3D_M = 1.25;

  // =========================================================================
  // Destination coordinates in the Unity world frame (X/Z), copied from the
  // official EnvironmentInfo01.json.  They must not be used directly as ROS
  // odom X/Y: the HSR starts at Unity (-2.0, -3.5), with ROS +X along Unity
  // +Z and ROS +Y along Unity -X.
  // =========================================================================
  static constexpr double HSR_INITIAL_UNITY_X = -2.0;
  static constexpr double HSR_INITIAL_UNITY_Z = -3.5;
  const std::map<std::string, std::pair<double, double>> KNOWN_DESTINATIONS_UNITY = {
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

  struct PlacementProfile {
    // Height of the grasped-object/finger midpoint in odom.  Values are
    // centred in the official JudgeTriggerOn volume, or immediately above an
    // open JudgeTriggerIn container so gravity carries the object inside.
    double release_height;
    double surface_inset;
    double center_offset_x;
    double center_offset_y;
    bool drop_in;
  };

  // =========================================================================
  // State variables
  // =========================================================================
  int step_;
  bool is_started_  = false;
  bool is_finished_ = false;
  bool is_failed_   = false;
  std::string failed_detail_;
  bool received_pick_command_  = false;
  bool received_clean_command_ = false;
  bool received_yes_ = false;
  bool received_no_  = false;

  std::chrono::steady_clock::time_point state_enter_time_;
  bool state_timer_initialized_ = false;
  int sub_step_ = 0;
  int scan_index_ = 0;
  int point_again_count_ = 0;
  int point_align_retry_count_ = 0;
  bool ready_ack_sent_ = false;
  std::chrono::steady_clock::time_point last_ready_ack_time_;
  std::chrono::steady_clock::time_point last_pointing_recv_time_;
  std::chrono::steady_clock::time_point last_valid_pointing_time_;
  std::chrono::steady_clock::time_point observe_align_start_time_;
  std::chrono::steady_clock::time_point observe_scan_start_time_;
  std::chrono::steady_clock::time_point last_avatar_track_time_;
  bool observe_align_active_ = false;
  bool observe_scan_active_ = false;
  int grasp_align_stable_cycles_ = 0;
  int release_align_stable_cycles_ = 0;
  int release_height_adjustments_ = 0;
  int target_view_recovery_count_ = 0;
  bool fine_target_seen_ = false;
  bool target_aim_pose_valid_ = false;
  double target_aim_origin_pan_ = 0.0;
  double target_aim_origin_tilt_ = 0.0;
  double target_aim_pan_ = 0.0;
  double target_aim_tilt_ = 0.0;

  // Joint state
  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;

  // Nav2
  bool nav_goal_sent_    = false;
  bool nav_goal_reached_ = false;
  bool nav_goal_failed_  = false;
  bool nav2_available_   = false;
  bool nav2_enabled_     = true;

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  // ---- Vision state ----
  DetectedObjectArray latest_objects_;
  PointingDirection   latest_pointing_;
  PointingDirection   last_valid_pointing_;
  PointingDirection   locked_pointing_;
  bool has_new_objects_  = false;
  bool has_new_pointing_ = false;
  bool has_last_valid_pointing_ = false;
  bool has_locked_pointing_ = false;

  // Accumulated observations for voting
  std::vector<DetectedObjectArray> obs_objects_;
  std::vector<PointingDirection>   obs_pointings_;

  // Selected targets
  geometry_msgs::msg::Point target_position_;
  bool   target_valid_ = false;
  std::string target_class_;

  // The pointing-ray origin lies on the Avatar's forearm and is therefore a
  // useful world-space hint for turning the robot back toward the person
  // after it has rotated away to inspect the pointed object.
  geometry_msgs::msg::Point person_hint_position_;
  bool person_hint_valid_ = false;

  geometry_msgs::msg::Point dest_position_;
  bool   dest_valid_ = false;
  std::string dest_class_;
  geometry_msgs::msg::Point placement_position_;
  PlacementProfile placement_profile_{0.65, 0.12, 0.0, 0.0, false};
  bool placement_ready_ = false;

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_vision_enable_;

  rclcpp::Subscription<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>::SharedPtr sub_msg_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hsr_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<DetectedObjectArray>::SharedPtr sub_detected_objects_;
  rclcpp::Subscription<PointingDirection>::SharedPtr   sub_pointing_;

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
      } else if (step_ == WaitForPickCommand) {
        sendReadyAck("moderator retried handshake");
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring unexpected Are_you_ready? while in [%s]",
          stepName(step_).c_str());
      }
    }
    else if (message == MSG_PICK_IT_UP)     { received_pick_command_ = true; }
    else if (message == MSG_CLEAN_UP)       { received_clean_command_ = true; }
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

  void objectsCallback(const DetectedObjectArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    has_new_objects_ = true;
    if (step_ == ObserveTarget || step_ == WaitForCleanCommand ||
        step_ == ObserveDestination)
      obs_objects_.push_back(*msg);
  }

  void pointingCallback(const PointingDirection::SharedPtr msg)
  {
    latest_pointing_ = *msg;
    has_new_pointing_ = true;
    last_pointing_recv_time_ = std::chrono::steady_clock::now();
    if (msg->is_valid) {
      last_valid_pointing_ = *msg;
      has_last_valid_pointing_ = true;
      last_valid_pointing_time_ = last_pointing_recv_time_;
    }
    if (step_ == ObserveTarget || step_ == WaitForCleanCommand ||
        step_ == ObserveDestination)
      obs_pointings_.push_back(*msg);
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
  // At the 0.75--0.85 m observation stand-off, a floor object is below the
  // old -0.85 rad optical axis.  Look farther down so the selected object can
  // be refined before the arm enters the image.
  void headLookAtFloor(){ moveHead(0.0, -1.05, 1.0); }

  bool aimHeadAlongImagePointing(const PointingDirection &pointing)
  {
    // Aim once at the point where the PIP/forearm -> fingertip line exits the
    // current image.  This brings the region immediately beyond the finger
    // into view without assuming that Avatar->object yaw equals robot->object
    // yaw and without performing a symmetric search.
    constexpr double width = 640.0;
    constexpr double height = 480.0;
    constexpr double margin = 16.0;
    constexpr double fx = 554.0;
    constexpr double fy = 554.0;
    constexpr double cx = 320.0;
    constexpr double cy = 240.0;

    const double dx = pointing.point_pixel_x - pointing.wrist_pixel_x;
    const double dy = pointing.point_pixel_y - pointing.wrist_pixel_y;
    if (std::hypot(dx, dy) < 8.0) {
      RCLCPP_WARN(this->get_logger(),
                  "[ObserveTarget] Image pointing segment is too short to aim head");
      return false;
    }

    double edge_scale = std::numeric_limits<double>::infinity();
    const auto consider = [&edge_scale](double value) {
      if (value > 0.0 && std::isfinite(value)) {
        edge_scale = std::min(edge_scale, value);
      }
    };
    if (dx > 1e-6) {
      consider((width - margin - pointing.point_pixel_x) / dx);
    } else if (dx < -1e-6) {
      consider((margin - pointing.point_pixel_x) / dx);
    }
    if (dy > 1e-6) {
      consider((height - margin - pointing.point_pixel_y) / dy);
    } else if (dy < -1e-6) {
      consider((margin - pointing.point_pixel_y) / dy);
    }
    if (!std::isfinite(edge_scale)) return false;

    // Look slightly beyond the old image boundary.  Capping the extrapolation
    // prevents a noisy short finger segment from commanding a joint limit.
    edge_scale = std::clamp(edge_scale + 0.35, 0.5, 10.0);
    // A nearly frontal arm has very little 2-D extent, so a few pixels of
    // landmark noise can extrapolate far outside the image (one recorded run
    // produced u=-689 and drove the head directly to its joint limit).  Keep
    // the directed motion, but bound both the virtual pixel and the change
    // from the current head pose.  The successful side-on behaviour remains
    // unchanged because its commands are already inside these bounds.
    const double look_u = std::clamp(
      pointing.point_pixel_x + edge_scale * dx,
      -0.25 * width, 1.25 * width);
    const double look_v = std::clamp(
      pointing.point_pixel_y + edge_scale * dy,
      -0.20 * height, 1.20 * height);
    const double image_yaw = std::atan2(look_u - cx, fx);
    const double image_pitch = std::atan2(look_v - cy, fy);
    const double current_pan = getJointPosition("head_pan_joint");
    const double current_tilt = getJointPosition("head_tilt_joint");
    const double target_pan = std::clamp(
      current_pan + std::clamp(-image_yaw, -0.70, 0.70), -1.2, 1.2);
    const double target_tilt = std::clamp(
      current_tilt + std::clamp(-image_pitch, -0.55, 0.55), -1.05, 0.35);
    target_aim_origin_pan_ = current_pan;
    target_aim_origin_tilt_ = current_tilt;
    target_aim_pan_ = target_pan;
    target_aim_tilt_ = target_tilt;
    target_aim_pose_valid_ = true;
    moveHead(target_pan, target_tilt, 0.8);
    RCLCPP_INFO(this->get_logger(),
                "[ObserveTarget] One-shot finger-directed head aim: "
                "pixel=(%.0f,%.0f) pan=%.2f tilt=%.2f",
                look_u, look_v, target_pan, target_tilt);
    return true;
  }

  void headScanStep(const std::vector<HeadPose> &pattern, int idx)
  {
    int i = idx % static_cast<int>(pattern.size());
    moveHead(pattern[i].pan, pattern[i].tilt, 1.0);
  }

  void trackAvatarWithHead()
  {
    if (!has_new_pointing_ || !latest_pointing_.person_is_valid) return;

    // Once a gesture is captured, hold the view until the delayed Unity
    // command arrives instead of moving the camera away from the finger ray.
    if (has_last_valid_pointing_) {
      const double gesture_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_valid_pointing_time_).count();
      if (gesture_age <= POINTING_MEMORY_SEC) return;
    }

    const auto now_time = std::chrono::steady_clock::now();
    if (last_avatar_track_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now_time - last_avatar_track_time_).count() <
          AVATAR_TRACK_INTERVAL) {
      return;
    }

    // Intrinsics reported by the current scene: fx=fy=554, cx=320, cy=240.
    // Positive image x is right while positive head pan is left. Negative
    // head tilt looks down, matching positive image y.
    constexpr double fx = 554.0, fy = 554.0, cx = 320.0, cy = 240.0;
    const double image_yaw = std::atan2(latest_pointing_.person_pixel_x - cx, fx);
    const double image_pitch = std::atan2(latest_pointing_.person_pixel_y - cy, fy);
    const double current_pan = getJointPosition("head_pan_joint");
    const double current_tilt = getJointPosition("head_tilt_joint");
    const double target_pan = std::clamp(current_pan - image_yaw, -1.2, 1.2);
    const double target_tilt = std::clamp(current_tilt - image_pitch, -0.7, 0.35);
    moveHead(target_pan, target_tilt, 0.3);
    last_avatar_track_time_ = now_time;
  }

  bool hasRecentVisibleAvatar(double max_age_sec = 0.75) const
  {
    if (!has_new_pointing_ || !latest_pointing_.person_is_valid) return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_pointing_recv_time_).count();
    return age <= max_age_sec;
  }

  void reacquireAvatarForDestination()
  {
    if (hasRecentVisibleAvatar()) {
      stopBase();
      trackAvatarWithHead();
      return;
    }

    if (!person_hint_valid_) {
      stopBase();
      return;
    }

    const double yaw_error = angleToPoint(
      person_hint_position_.x, person_hint_position_.y);
    if (std::abs(yaw_error) <= 0.12) {
      stopBase();
      return;
    }

    const double angular = std::clamp(yaw_error * 1.4, -0.40, 0.40);
    turnBase(angular);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[WaitForClean] Turning back toward Avatar hint err=%.2f cmd=%.2f",
      yaw_error, angular);
  }

  // =========================================================================
  // Gripper
  // =========================================================================
  void operateHand(bool grasp)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"hand_motor_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {grasp ? -0.105 : 1.239};
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
    enableVision(false);
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
  void enableVision(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    pub_vision_enable_->publish(m);
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

  static bool isLikelyPortableClass(const std::string &class_name)
  {
    static const std::vector<std::string> portable_classes = {
      "bottle", "cup", "wine glass", "bowl", "book", "clock",
      "vase", "teddy bear", "sports ball", "cell phone",
      "remote", "toothbrush",
      // Official candidate set in the current Interactive Cleanup Unity
      // EnvironmentInfo01..05 configurations.
      "bear_doll", "canned_juice", "dog_doll", "filled_ketchup",
      "filled_plastic_bottle", "hourglass", "rabbit_doll", "sauce",
      "soysauce", "spray_bottle", "sugar", "toy_penguin", "tumbler"
    };
    return std::find(
      portable_classes.begin(), portable_classes.end(), class_name) !=
      portable_classes.end();
  }

  static bool isLikelyDestinationClass(const std::string &class_name)
  {
    static const std::vector<std::string> destination_classes = {
      "bench", "chair", "couch", "bed", "dining table", "potted plant",
      "sink", "toilet", "refrigerator", "oven", "microwave", "tv",
      "suitcase"
    };
    return std::find(
      destination_classes.begin(), destination_classes.end(), class_name) !=
      destination_classes.end();
  }

  /**
   * Select a destination from the Avatar's finger ray.
   *
   * The vision node publishes an index-finger PIP→tip ray in odom.  The
   * environment file publishes Unity-world X/Z coordinates, so every known
   * destination is first converted into the HSR's odom frame.  Requiring
   * repeated, forward-facing rays stops the old behaviour of choosing a bin
   * merely because it is near the Avatar.
   */
  bool selectDestinationByPointingRay()
  {
    struct DestinationVote { int samples{0}; double ray_distance_sum{0.0}; };
    std::map<std::string, DestinationVote> votes;

    int valid_ray_count = 0;
    int reversed_ray_count = 0;
    constexpr double MAX_DEST_RAY_DISTANCE_M = 1.0;
    // The object observations were cleared as soon as the target was locked,
    // so the newest high-confidence rays now belong to the destination
    // gesture. MediaPipe can insert invalid frames between good estimates;
    // use up to the six newest valid rays instead of requiring a contiguous
    // burst that may collapse to a single sample.
    std::vector<const PointingDirection *> recent_valid_rays;
    for (int i = static_cast<int>(obs_pointings_.size()) - 1;
         i >= 0 && recent_valid_rays.size() < 6; --i) {
      const auto &pt = obs_pointings_[i];
      if (pt.is_valid && pt.confidence >= 0.60) {
        recent_valid_rays.push_back(&pt);
      }
    }

    for (const auto *pt_ptr : recent_valid_rays) {
      const auto &pt = *pt_ptr;
      const double direction_norm = std::sqrt(
        pt.direction.x * pt.direction.x + pt.direction.y * pt.direction.y +
        pt.direction.z * pt.direction.z);
      if (direction_norm < 1e-3) continue;

      const double ray_dx = pt.direction.x;
      const double ray_dy = pt.direction.y;
      if (std::hypot(ray_dx, ray_dy) < 1e-3) continue;
      valid_ray_count++;

      double best_forward_distance = std::numeric_limits<double>::infinity();
      double best_reverse_distance = std::numeric_limits<double>::infinity();
      for (const auto &[name, unity_coordinate] : KNOWN_DESTINATIONS_UNITY) {
        (void)name;
        const double odom_x = unity_coordinate.second - HSR_INITIAL_UNITY_Z;
        const double odom_y = -(unity_coordinate.first - HSR_INITIAL_UNITY_X);
        best_forward_distance = std::min(
          best_forward_distance,
          pointRayDist2D(
            odom_x, odom_y, pt.origin.x, pt.origin.y, ray_dx, ray_dy));
        best_reverse_distance = std::min(
          best_reverse_distance,
          pointRayDist2D(
            odom_x, odom_y, pt.origin.x, pt.origin.y, -ray_dx, -ray_dy));
      }
      const bool reverse_ray =
        best_forward_distance > MAX_DEST_RAY_DISTANCE_M &&
        best_reverse_distance <= MAX_DEST_RAY_DISTANCE_M;
      const double ray_sign = reverse_ray ? -1.0 : 1.0;
      if (reverse_ray) ++reversed_ray_count;

      for (const auto &[name, unity_coordinate] : KNOWN_DESTINATIONS_UNITY) {
        const double odom_x = unity_coordinate.second - HSR_INITIAL_UNITY_Z;
        const double odom_y = -(unity_coordinate.first - HSR_INITIAL_UNITY_X);
        const double distance = pointRayDist2D(
          odom_x, odom_y, pt.origin.x, pt.origin.y,
          ray_sign * ray_dx, ray_sign * ray_dy);
        if (distance < 9998.0) {
          auto &vote = votes[name];
          vote.samples++;
          vote.ray_distance_sum += distance;
        }
      }
    }

    if (valid_ray_count < 1) {
      RCLCPP_WARN(this->get_logger(),
                  "[PointDest] Need a valid high-confidence 3-D finger ray; received %d",
                  valid_ray_count);
      return false;
    }

    constexpr int MIN_DEST_POINT_SAMPLES = 1;
    std::string best_name;
    double best_average_distance = std::numeric_limits<double>::infinity();
    int best_samples = 0;
    for (const auto &[name, vote] : votes) {
      if (vote.samples < MIN_DEST_POINT_SAMPLES) continue;
      const double average_distance = vote.ray_distance_sum / vote.samples;
      if (average_distance < best_average_distance) {
        best_name = name;
        best_average_distance = average_distance;
        best_samples = vote.samples;
      }
    }

    if (best_name.empty() || best_average_distance > MAX_DEST_RAY_DISTANCE_M) {
      RCLCPP_WARN(this->get_logger(),
                  "[PointDest] No destination is consistently near the finger ray "
                  "(best=%.2fm; limit=%.2fm)",
                  best_average_distance, MAX_DEST_RAY_DISTANCE_M);
      return false;
    }

    const auto &unity_coordinate = KNOWN_DESTINATIONS_UNITY.at(best_name);
    dest_position_.x = unity_coordinate.second - HSR_INITIAL_UNITY_Z;
    dest_position_.y = -(unity_coordinate.first - HSR_INITIAL_UNITY_X);
    dest_position_.z = 0.0;
    dest_valid_ = true;
    dest_class_ = best_name;
    if (reversed_ray_count > 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PointDest] Resolved depth-direction ambiguity for %d/%d rays",
        reversed_ray_count, valid_ray_count);
    }
    RCLCPP_INFO(this->get_logger(),
                "[PointDest] Selected '%s' at odom=(%.2f, %.2f), "
                "Unity=(%.2f, %.2f): ray=%.2fm from %d samples",
                dest_class_.c_str(), dest_position_.x, dest_position_.y,
                unity_coordinate.first, unity_coordinate.second,
                best_average_distance, best_samples);
    return true;
  }

  /**
   * Select the target/destination from accumulated observations.
   * @param is_dest true → picking destination (prefer furniture classes)
   * @return true if a valid target was found
   */
  bool selectFromObservations(bool is_dest)
  {
    if (obs_objects_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No detections during observation");
      return false;
    }

    struct Candidate {
      std::string name;
      double x, y, z;
      int votes;
      double score_sum;
      double best_ray3d;
      double best_ray2d;
      bool portable_like;
      bool destination_like;
    };
    std::map<std::string, Candidate> cands;

    for (size_t fi = 0; fi < obs_objects_.size(); ++fi) {
      const auto &frame = obs_objects_[fi];

      // Find the temporally-closest pointing for this frame
      const PointingDirection *pt = nullptr;
      if (!obs_pointings_.empty()) {
        auto frame_t = rclcpp::Time(frame.header.stamp);
        double best_dt = 1e9;
        for (auto &p : obs_pointings_) {
          double dt = std::abs((rclcpp::Time(p.header.stamp) - frame_t).seconds());
          if (dt < best_dt) { best_dt = dt; pt = &p; }
        }
        if (best_dt > 1.0) pt = nullptr;
      }

      for (const auto &obj : frame.objects) {
        if (obj.class_name == "person") continue;
        if (!obj.has_3d_position) continue;

        double ray_d_2d = 9999.0;
        if (pt && pt->is_valid) {
          ray_d_2d = pointRayDist2D(
            obj.bbox_cx, obj.bbox_cy,
            pt->wrist_pixel_x, pt->wrist_pixel_y,
            pt->point_pixel_x - pt->wrist_pixel_x,
            pt->point_pixel_y - pt->wrist_pixel_y);
        }

        double ray_d_3d = 9999.0;
        if (pt && pt->is_valid) {
          ray_d_3d = pointRayDist3D(
            obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
            pt->origin.x, pt->origin.y, pt->origin.z,
            pt->direction.x, pt->direction.y, pt->direction.z);
        }

        const bool portable_like = isLikelyPortableClass(obj.class_name);
        const bool destination_like = isLikelyDestinationClass(obj.class_name);

        // Build a spatial key to avoid merging distant same-class objects
        std::string key = obj.class_name + "_" +
            std::to_string(int(obj.position_3d.x * 2)) + "_" +
            std::to_string(int(obj.position_3d.y * 2));

        if (cands.find(key) == cands.end()) {
          cands[key] = {obj.class_name,
                        obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
                        0, 0.0, 9999.0, 9999.0,
                        portable_like, destination_like};
        }
        auto &c = cands[key];

        double semantic_weight = 1.0;
        if (is_dest) {
          if (destination_like) semantic_weight *= 1.8;
          if (portable_like)    semantic_weight *= 0.25;

          // Destinations should usually differ from the currently selected target
          // and not be the same supporting furniture right next to it.
          if (target_valid_) {
            double dist_to_target = std::hypot(
              obj.position_3d.x - target_position_.x,
              obj.position_3d.y - target_position_.y);
            if (dist_to_target < 0.8) { semantic_weight *= 0.35; }
          }
        } else {
          if (portable_like)    semantic_weight *= 1.6;
          if (destination_like) semantic_weight *= 0.15;

          // Large detections are often support surfaces rather than graspable items.
          int area = obj.bbox_w * obj.bbox_h;
          if (area > 50000) { semantic_weight *= 0.6; }
        }

        double pointing_weight_3d = 0.7;
        if (ray_d_3d < 9998.0) {
          double scale = is_dest ? 0.8 : 0.45;
          pointing_weight_3d = 1.0 / (1.0 + ray_d_3d / scale);
          pointing_weight_3d *= 1.5;  // Prefer 3D-consistent candidates.
        }

        double pointing_weight_2d = 0.8;
        if (ray_d_2d < 9998.0) {
          double scale_px = is_dest ? 120.0 : 80.0;
          pointing_weight_2d = 1.0 / (1.0 + ray_d_2d / scale_px);
        }

        double vote_weight =
          semantic_weight * pointing_weight_3d * pointing_weight_2d;

        c.votes++;
        c.score_sum += obj.confidence * vote_weight;
        c.best_ray3d = std::min(c.best_ray3d, ray_d_3d);
        c.best_ray2d = std::min(c.best_ray2d, ray_d_2d);

        // Running average of position
        double n = static_cast<double>(c.votes);
        c.x = c.x * (n - 1) / n + obj.position_3d.x / n;
        c.y = c.y * (n - 1) / n + obj.position_3d.y / n;
        c.z = c.z * (n - 1) / n + obj.position_3d.z / n;
      }
    }

    if (cands.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid candidates after filtering");
      return false;
    }

    bool has_good_3d_candidate = false;
    bool has_non_destination_candidate = false;
    bool has_destination_like_candidate = false;
    for (const auto &[k, c] : cands) {
      if (c.best_ray3d < (is_dest ? 0.8 : 0.45)) {
        has_good_3d_candidate = true;
      }
      if (!c.destination_like) {
        has_non_destination_candidate = true;
      }
      if (c.destination_like) {
        has_destination_like_candidate = true;
      }
    }

    // Pick the candidate with highest weighted confidence sum
    const Candidate *best = nullptr;
    double best_score = -1.0;
    for (auto &[k, c] : cands) {
      if (!is_dest && has_non_destination_candidate && c.destination_like) {
        continue;
      }
      if (is_dest && has_destination_like_candidate &&
          c.portable_like && !c.destination_like) {
        continue;
      }
      if (has_good_3d_candidate &&
          c.best_ray3d > (is_dest ? 1.2 : 0.7)) {
        continue;
      }

      if (c.score_sum > best_score) {
        best_score = c.score_sum;
        best = &c;
      }
    }

    if (!best) return false;

    auto &pos   = is_dest ? dest_position_ : target_position_;
    auto &valid = is_dest ? dest_valid_     : target_valid_;
    auto &cls   = is_dest ? dest_class_     : target_class_;

    pos.x = best->x;
    pos.y = best->y;
    pos.z = best->z;
    valid = true;
    cls   = best->name;

    RCLCPP_INFO(this->get_logger(),
                "Selected %s: class='%s' pos=(%.2f, %.2f, %.2f) votes=%d ray3d=%.2f ray2d=%.1f score=%.3f",
                is_dest ? "DESTINATION" : "TARGET",
                cls.c_str(), pos.x, pos.y, pos.z, best->votes,
                best->best_ray3d, best->best_ray2d, best->score_sum);

    return true;
  }

  // Select using the whole observation window. Detection persistence is the
  // primary signal; 3-D ray proximity is a soft weight. This is robust to a
  // single depth outlier passing very close to the ray and to a real target
  // landing just outside a brittle hard distance threshold.
  bool selectStablePointedTarget()
  {
    struct Candidate {
      std::string name;
      double x, y, z;
      int frames;
      double confidence_sum;
      double ray_distance_sum;
      double best_ray_distance;
    };
    std::map<std::string, Candidate> candidates;

    // MediaPipe's depth axis and a camera optical frame have opposite Z
    // conventions.  The vision node now converts that explicitly, but depth
    // fallbacks can still be ambiguous for a nearly frontal, foreshortened
    // finger.  Only reverse a ray when the published direction has zero
    // portable-object support and its opposite has stable multi-frame
    // support.  This handles the observed all-9999m failure without turning
    // the pointing ray into an unrestricted line.
    int forward_support = 0;
    int reverse_support = 0;
    for (const auto &frame : obs_objects_) {
      if (!has_locked_pointing_) continue;
      for (const auto &obj : frame.objects) {
        if (obj.class_name == "person" || !obj.has_3d_position ||
            obj.confidence < STABLE_TARGET_MIN_CONFIDENCE ||
            !isLikelyPortableClass(obj.class_name)) {
          continue;
        }
        const double forward_distance = pointRayDist3D(
          obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
          locked_pointing_.origin.x, locked_pointing_.origin.y,
          locked_pointing_.origin.z, locked_pointing_.direction.x,
          locked_pointing_.direction.y, locked_pointing_.direction.z);
        const double reverse_distance = pointRayDist3D(
          obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
          locked_pointing_.origin.x, locked_pointing_.origin.y,
          locked_pointing_.origin.z, -locked_pointing_.direction.x,
          -locked_pointing_.direction.y, -locked_pointing_.direction.z);
        if (forward_distance <= 1.5) ++forward_support;
        if (reverse_distance <= 1.5) ++reverse_support;
      }
    }
    const bool reverse_ray =
      forward_support == 0 && reverse_support >= STABLE_TARGET_MIN_FRAMES;
    const double ray_sign = reverse_ray ? -1.0 : 1.0;
    if (reverse_ray) {
      RCLCPP_WARN(
        this->get_logger(),
        "[ObserveTarget] Resolving frontal pointing depth ambiguity: "
        "forward support=%d, reverse support=%d",
        forward_support, reverse_support);
    }

    for (const auto &frame : obs_objects_) {
      if (!has_locked_pointing_) continue;
      for (const auto &obj : frame.objects) {
        if (obj.class_name == "person" || !obj.has_3d_position ||
            obj.confidence < STABLE_TARGET_MIN_CONFIDENCE ||
            !isLikelyPortableClass(obj.class_name)) {
          continue;
        }

        const double ray_distance = pointRayDist3D(
          obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
          locked_pointing_.origin.x, locked_pointing_.origin.y,
          locked_pointing_.origin.z,
          ray_sign * locked_pointing_.direction.x,
          ray_sign * locked_pointing_.direction.y,
          ray_sign * locked_pointing_.direction.z);
        if (ray_distance > 1.5) continue;

        auto result = candidates.emplace(
          obj.class_name,
          Candidate{obj.class_name, obj.position_3d.x, obj.position_3d.y,
                    obj.position_3d.z, 0, 0.0, 0.0,
                    std::numeric_limits<double>::infinity()});
        auto &candidate = result.first->second;
        const double n = static_cast<double>(candidate.frames);
        candidate.x = (candidate.x * n + obj.position_3d.x) / (n + 1.0);
        candidate.y = (candidate.y * n + obj.position_3d.y) / (n + 1.0);
        candidate.z = (candidate.z * n + obj.position_3d.z) / (n + 1.0);
        candidate.frames++;
        candidate.confidence_sum += obj.confidence;
        candidate.ray_distance_sum += ray_distance;
        candidate.best_ray_distance = std::min(
          candidate.best_ray_distance, ray_distance);
      }
    }

    const Candidate *best = nullptr;
    double best_score = -1.0;
    for (const auto &[key, candidate] : candidates) {
      if (candidate.frames < STABLE_TARGET_MIN_FRAMES ||
          candidate.best_ray_distance > STABLE_TARGET_MAX_RAY_3D_M) continue;
      const double average_confidence =
        candidate.confidence_sum / candidate.frames;
      const double average_ray_distance =
        candidate.ray_distance_sum / candidate.frames;
      const double score =
        std::pow(static_cast<double>(candidate.frames), 1.25) *
        average_confidence * std::exp(-average_ray_distance / 1.2);
      if (score > best_score) {
        best_score = score;
        best = &candidate;
      }
    }
    if (best == nullptr) {
      return false;
    }

    target_position_.x = best->x;
    target_position_.y = best->y;
    target_position_.z = best->z;
    target_valid_ = true;
    target_class_ = best->name;
    RCLCPP_INFO(this->get_logger(),
                "[ObserveTarget] Stable pointed target: class='%s' pos=(%.2f, %.2f, %.2f) frames=%d avg_ray=%.2fm score=%.2f",
                target_class_.c_str(), target_position_.x, target_position_.y,
                target_position_.z, best->frames,
                best->ray_distance_sum / best->frames, best_score);
    return true;
  }

  void logObservationSummary(const char *label) const
  {
    std::map<std::string, int> counts;
    int total_objects = 0;
    int total_objects_3d = 0;

    for (const auto &frame : obs_objects_) {
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
                  label, obs_objects_.size());
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
                label, obs_objects_.size(), total_objects, total_objects_3d,
                oss.str().c_str());
  }

  bool hasObservedNonPersonObject() const
  {
    for (const auto &frame : obs_objects_) {
      for (const auto &obj : frame.objects) {
        if (obj.class_name != "person") return true;
      }
    }
    return false;
  }

  void logPointingRayDiagnostics(const char *label) const
  {
    if (!has_locked_pointing_) {
      RCLCPP_WARN(this->get_logger(), "[%s] Ray diagnostics: no locked ray", label);
      return;
    }

    std::map<std::string, double> minimum_distance;
    for (const auto &frame : obs_objects_) {
      for (const auto &obj : frame.objects) {
        if (!obj.has_3d_position || obj.class_name == "person") continue;
        const double distance = pointRayDist3D(
          obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
          locked_pointing_.origin.x, locked_pointing_.origin.y,
          locked_pointing_.origin.z, locked_pointing_.direction.x,
          locked_pointing_.direction.y, locked_pointing_.direction.z);
        auto it = minimum_distance.find(obj.class_name);
        if (it == minimum_distance.end() || distance < it->second) {
          minimum_distance[obj.class_name] = distance;
        }
      }
    }

    std::vector<std::pair<std::string, double>> ranked(
      minimum_distance.begin(), minimum_distance.end());
    std::sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
      return a.second < b.second;
    });
    std::ostringstream oss;
    const size_t limit = std::min<size_t>(ranked.size(), 8);
    for (size_t i = 0; i < limit; ++i) {
      if (i != 0) oss << ", ";
      oss << ranked[i].first << ":" << ranked[i].second << "m";
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Locked ray origin=(%.2f,%.2f,%.2f) dir=(%.2f,%.2f,%.2f) nearest=[%s]",
      label, locked_pointing_.origin.x, locked_pointing_.origin.y,
      locked_pointing_.origin.z, locked_pointing_.direction.x,
      locked_pointing_.direction.y, locked_pointing_.direction.z,
      oss.str().c_str());
  }

  // =========================================================================
  // Robot pose helpers (odom frame)
  // =========================================================================
  struct RobotPose { double x, y, yaw; bool valid; };

  RobotPose getRobotPose()
  {
    RobotPose rp{0, 0, 0, false};
    try {
      auto tf = tf_buffer_->lookupTransform("odom", "base_footprint",
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

  double angleToPoint(double tx, double ty)
  {
    auto rp = getRobotPose();
    if (!rp.valid) return 0.0;
    double a = std::atan2(ty - rp.y, tx - rp.x) - rp.yaw;
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  double distToPoint(double tx, double ty)
  {
    auto rp = getRobotPose();
    if (!rp.valid) return 0.0;
    return std::hypot(tx - rp.x, ty - rp.y);
  }

  bool getGripperMidpoint(geometry_msgs::msg::Point &point)
  {
    try {
      const auto left_tip = tf_buffer_->lookupTransform(
        "odom", "hand_l_finger_tip_frame", tf2::TimePointZero);
      const auto right_tip = tf_buffer_->lookupTransform(
        "odom", "hand_r_finger_tip_frame", tf2::TimePointZero);
      const auto left_root = tf_buffer_->lookupTransform(
        "odom", "hand_l_distal_link", tf2::TimePointZero);
      const auto right_root = tf_buffer_->lookupTransform(
        "odom", "hand_r_distal_link", tf2::TimePointZero);

      // Unity's five trigger strips on each finger are centred about 79% of
      // the way from distal-link origin to fingertip (mean local z ~=30 mm,
      // fingertip z=37.9 mm).  Align this collision region, not the foremost
      // fingertip point, which otherwise passes beyond the object.
      constexpr double trigger_fraction = 0.79;
      const auto interp = [trigger_fraction](double root, double tip) {
        return root + trigger_fraction * (tip - root);
      };
      const double left_x = interp(left_root.transform.translation.x,
                                   left_tip.transform.translation.x);
      const double left_y = interp(left_root.transform.translation.y,
                                   left_tip.transform.translation.y);
      const double left_z = interp(left_root.transform.translation.z,
                                   left_tip.transform.translation.z);
      const double right_x = interp(right_root.transform.translation.x,
                                    right_tip.transform.translation.x);
      const double right_y = interp(right_root.transform.translation.y,
                                    right_tip.transform.translation.y);
      const double right_z = interp(right_root.transform.translation.z,
                                    right_tip.transform.translation.z);
      point.x = 0.5 * (left_x + right_x);
      point.y = 0.5 * (left_y + right_y);
      point.z = 0.5 * (left_z + right_z);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Gripper TF unavailable: %s", ex.what());
      return false;
    }
  }

  PlacementProfile getPlacementProfile(const std::string &name) const
  {
    // These heights come from the JudgeTriggerOn/JudgeTriggerIn child
    // colliders in the official Layout2019IC01 prefab.  The cardboard-box
    // trigger is also offset from its root; all other trigger centres are
    // horizontally coincident with the configured destination coordinate.
    if (name == "square_low_table") {
      return {0.59, 0.12, 0.0, 0.0, false};
    }
    if (name == "wagon_1" || name == "wagon_2") {
      return {0.88, 0.10, 0.0, 0.0, false};
    }
    if (name == "white_side_table_1" || name == "white_side_table_2") {
      return {0.66, 0.14, 0.0, 0.0, false};
    }
    if (name == "wooden_shelf") {
      return {0.82, 0.10, 0.0, 0.0, false};
    }
    if (name == "cardboard_box") {
      // Unity local (-0.135, *, +0.001), rotated +10 degrees at the root,
      // gives world (dX,dZ)=(-0.133,+0.024).  Unity X/Z vectors map to
      // odom (dX,dY)=(dZ,-dX).
      return {0.32, 0.0, 0.024, 0.133, true};
    }
    if (name.rfind("trash_box_for_", 0) == 0) {
      return {0.55, 0.0, 0.0, 0.0, true};
    }
    return {0.65, 0.12, 0.0, 0.0, false};
  }

  bool preparePlacementTarget()
  {
    const auto robot = getRobotPose();
    if (!dest_valid_ || !robot.valid) return false;

    placement_profile_ = getPlacementProfile(dest_class_);
    const double center_x = dest_position_.x + placement_profile_.center_offset_x;
    const double center_y = dest_position_.y + placement_profile_.center_offset_y;
    const double from_center_x = robot.x - center_x;
    const double from_center_y = robot.y - center_y;
    const double norm = std::hypot(from_center_x, from_center_y);
    const double ux = norm > 1e-6 ? from_center_x / norm : 1.0;
    const double uy = norm > 1e-6 ? from_center_y / norm : 0.0;

    // On a surface, use a point slightly toward the robot.  It remains well
    // inside even the smallest official tabletop trigger and shortens the
    // required reach.  Containers must use their exact opening centre.
    placement_position_.x =
      center_x + placement_profile_.surface_inset * ux;
    placement_position_.y =
      center_y + placement_profile_.surface_inset * uy;
    placement_position_.z = placement_profile_.release_height;
    placement_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "[Placement] '%s' target=(%.2f, %.2f, %.2f) mode=%s",
      dest_class_.c_str(), placement_position_.x, placement_position_.y,
      placement_position_.z,
      placement_profile_.drop_in ? "drop-in" : "surface");
    return true;
  }

  static double normalizeAngle(double angle)
  {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  bool getRecentValidPointing(PointingDirection &pt, double max_age_sec = POINTING_MEMORY_SEC)
  {
    if (!has_last_valid_pointing_) {
      return false;
    }

    double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_valid_pointing_time_).count();
    if (age > max_age_sec) {
      return false;
    }

    double ground_norm = std::hypot(
      last_valid_pointing_.direction.x, last_valid_pointing_.direction.y);
    if (ground_norm < 1e-3) {
      return false;
    }

    pt = last_valid_pointing_;
    return true;
  }

  bool lockRecentPointing()
  {
    PointingDirection pt;
    if (!getRecentValidPointing(pt)) {
      return false;
    }
    locked_pointing_ = pt;
    has_locked_pointing_ = true;
    return true;
  }

  bool hasRecentImagePointing(double max_age_sec = POINTING_MAX_AGE_SEC) const
  {
    if (!has_new_pointing_ || !latest_pointing_.is_valid) {
      return false;
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_pointing_recv_time_).count();
    return age <= max_age_sec;
  }

  bool getPointingYawError(double &desired_yaw, double &yaw_error)
  {
    PointingDirection pt;
    if (getRecentValidPointing(pt)) {
      locked_pointing_ = pt;
      has_locked_pointing_ = true;
    } else if (has_locked_pointing_) {
      pt = locked_pointing_;
    } else {
      return false;
    }

    auto rp = getRobotPose();
    if (!rp.valid) {
      return false;
    }

    desired_yaw = std::atan2(pt.direction.y, pt.direction.x);
    yaw_error = normalizeAngle(desired_yaw - rp.yaw);
    return true;
  }

  void beginObservationAlignment(const char *label)
  {
    observe_align_active_ = true;
    observe_scan_active_ = false;
    observe_align_start_time_ = std::chrono::steady_clock::now();
    stopBase();
    headCenter();
    RCLCPP_INFO(this->get_logger(), "[%s] Starting base alignment from pointing", label);
  }

  void beginObservationScan(const char *label)
  {
    observe_align_active_ = false;
    observe_scan_active_ = true;
    observe_scan_start_time_ = std::chrono::steady_clock::now();
    scan_index_ = 0;
    stopBase();
    headLookDown();
    RCLCPP_INFO(this->get_logger(), "[%s] Starting local head scan", label);
  }

  void beginPointingHold(const char *label)
  {
    // Do not move the camera after it has captured the Avatar's finger.  A
    // scan makes the Avatar leave the image and invalidates the image-space
    // PIP→tip ray before it can be matched with object detections.
    observe_align_active_ = false;
    observe_scan_active_ = false;
    stopBase();
    RCLCPP_INFO(this->get_logger(),
                "[%s] Holding current view while confirming finger direction", label);
  }

  void beginPointingWait(const char *label)
  {
    // The Unity Avatar sends its command before (or while) it begins the
    // visible gesture.  Keep the current camera view instead of starting a
    // scan that would move the Avatar out of frame before the gesture arrives.
    observe_align_active_ = false;
    observe_scan_active_ = false;
    stopBase();
    RCLCPP_INFO(this->get_logger(),
                "[%s] Waiting for pointing gesture without moving camera", label);
  }

  double observeAlignElapsed() const
  {
    return observe_align_active_
      ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - observe_align_start_time_).count()
      : 0.0;
  }

  double observeScanElapsed() const
  {
    return observe_scan_active_
      ? std::chrono::duration<double>(
          std::chrono::steady_clock::now() - observe_scan_start_time_).count()
      : 0.0;
  }

  bool runPointingAlignment(const char *label)
  {
    double desired_yaw = 0.0;
    double yaw_error = 0.0;
    if (!getPointingYawError(desired_yaw, yaw_error)) {
      RCLCPP_WARN(this->get_logger(),
                  "[%s] No valid recent pointing, fallback to head scan", label);
      stopBase();
      return true;
    }

    if (std::abs(yaw_error) < POINT_ALIGN_ANGLE_THRESHOLD) {
      stopBase();
      RCLCPP_INFO(this->get_logger(),
                  "[%s] Pointing alignment complete yaw=%.2f err=%.2f",
                  label, desired_yaw, yaw_error);
      return true;
    }

    double angular = std::clamp(yaw_error * 1.6,
                                -POINT_ALIGN_MAX_ANGULAR,
                                 POINT_ALIGN_MAX_ANGULAR);
    turnBase(angular);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[%s] Aligning to pointing yaw=%.2f err=%.2f cmd=%.2f",
      label, desired_yaw, yaw_error, angular);
    return false;
  }

  // =========================================================================
  // Nav2
  // =========================================================================
  void sendNavGoal(double x, double y, double yaw)
  {
    if (nav_goal_sent_) return;
    if (!nav2_enabled_) {
      nav2_available_ = false;
      nav_goal_failed_ = true;
      RCLCPP_INFO(this->get_logger(), "Nav2 disabled; using direct approach");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server (up to 10s)...");
    if (!nav_client_->wait_for_action_server(10s)) {
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
    opts.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
      nav_goal_sent_ = false;
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        nav_goal_reached_ = true;
      } else {
        nav_goal_failed_ = true;
      }
    };

    nav_client_->async_send_goal(goal, opts);
    nav_goal_sent_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
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
    scan_index_ = 0;
    point_again_count_ = 0;
    point_align_retry_count_ = 0;
    ready_ack_sent_ = false;
    has_new_objects_ = false;
    has_new_pointing_ = false;
    has_last_valid_pointing_ = false;
    has_locked_pointing_ = false;
    target_valid_ = false;
    dest_valid_ = false;
    placement_ready_ = false;
    release_align_stable_cycles_ = 0;
    release_height_adjustments_ = 0;
    target_view_recovery_count_ = 0;
    target_aim_pose_valid_ = false;
    person_hint_valid_ = false;
    obs_objects_.clear();
    obs_pointings_.clear();
    observe_align_active_ = false;
    observe_scan_active_ = false;
    resetNavState();
  }

  std::string stepName(int s)
  {
    switch (s) {
      case Initialize:       return "Initialize";
      case Ready:            return "Ready";
      case WaitForPickCommand:  return "WaitForPick";
      case ObserveTarget:    return "ObserveTarget";
      case WaitForCleanCommand: return "WaitForClean";
      case ObserveDestination: return "ObserveDest";
      case MoveToObject:     return "MoveToObject";
      case GraspObject:      return "GraspObject";
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
      enableVision(false);
      RCLCPP_INFO(this->get_logger(), "[Init] Waiting for Are_you_ready?");
      step_ = Ready;
      break;
    }

    // -----------------------------------------------------------------
    case Ready: {
      if (is_started_) {
        armObserve();
        headCenter();
        sendReadyAck("transition to task start");
        enableVision(true);
        changeStep(WaitForPickCommand);
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForPickCommand: {
      trackAvatarWithHead();
      if (received_pick_command_) {
        RCLCPP_INFO(this->get_logger(), "[WaitForPick] Got Pick_it_up! → observing target");
        received_pick_command_ = false;
        obs_objects_.clear();
        obs_pointings_.clear();
        scan_index_ = 0;
        target_view_recovery_count_ = 0;
        target_aim_pose_valid_ = false;
        changeStep(ObserveTarget);
      } else if (state_timer_initialized_ && elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case ObserveTarget: {
      if (sub_step_ == 0) {
        if (lockRecentPointing()) {
          // The pointing vector is Avatar -> object, not robot -> object.
          // Turning the base to that vector is geometrically wrong and used
          // to produce the unnecessary opposite-side scan seen in Unity.
          if (aimHeadAlongImagePointing(locked_pointing_)) {
            // Ignore frames captured while the camera is moving.  A new
            // voting window starts after this single directed motion settles.
            obs_objects_.clear();
            state_enter_time_ = std::chrono::steady_clock::now();
            sub_step_ = 1;
          } else {
            beginPointingHold("ObserveTarget");
            sub_step_ = 2;
          }
        } else {
          beginPointingWait("ObserveTarget");
          sub_step_ = 4;
        }
      } else if (sub_step_ == 4) {
        trackAvatarWithHead();
        if (lockRecentPointing()) {
          RCLCPP_INFO(this->get_logger(),
                      "[ObserveTarget] Pointing acquired; holding current view");
          // Give target voting a complete window from the moment the gesture
          // is seen. Move the head once along the visible finger extension;
          // the base remains stationary.
          state_enter_time_ = std::chrono::steady_clock::now();
          if (aimHeadAlongImagePointing(locked_pointing_)) {
            obs_objects_.clear();
            sub_step_ = 1;
          } else {
            beginPointingHold("ObserveTarget");
            sub_step_ = 2;
          }
        }
      } else if (sub_step_ == 1 && elapsed() >= 1.0) {
        // The commanded head trajectory is 0.8 s. Discard transition frames,
        // then keep this directed view fixed for stable multi-frame voting.
        obs_objects_.clear();
        beginPointingHold("ObserveTarget");
        state_enter_time_ = std::chrono::steady_clock::now();
        sub_step_ = 2;
      } else if (sub_step_ == 2 && elapsed() >= 3.0 &&
                 !hasObservedNonPersonObject() &&
                 target_aim_pose_valid_ && target_view_recovery_count_ == 0) {
        // A noisy 2-D finger extension can put the first one-shot view just
        // beyond the object.  Only when that view is completely empty, move
        // once to the midpoint of the same directed motion and include more
        // floor.  This is deliberately not the old symmetric left/right scan.
        const double recovery_pan =
          0.5 * (target_aim_origin_pan_ + target_aim_pan_);
        const double recovery_tilt = std::min(
          0.5 * (target_aim_origin_tilt_ + target_aim_tilt_), -0.25);
        moveHead(recovery_pan, recovery_tilt, 0.8);
        target_view_recovery_count_ = 1;
        obs_objects_.clear();
        RCLCPP_WARN(
          this->get_logger(),
          "[ObserveTarget] Directed view contains no objects; "
          "trying one reduced same-direction view pan=%.2f tilt=%.2f",
          recovery_pan, recovery_tilt);
        sub_step_ = 6;
        enterTimedState();
      } else if (sub_step_ == 6 && elapsed() >= 1.0) {
        // Discard frames produced while the recovery trajectory was moving.
        obs_objects_.clear();
        beginPointingHold("ObserveTarget");
        sub_step_ = 7;
        enterTimedState();
      }

      // Confirm as soon as a short multi-frame vote is stable while retaining
      // the exact view in which the finger and candidate objects were seen.
      if ((sub_step_ == 2 || sub_step_ == 7) &&
          elapsed() >= TARGET_EARLY_CONFIRM_DELAY &&
          selectStablePointedTarget()) {
        stopBase();
        if (has_locked_pointing_) {
          person_hint_position_ = locked_pointing_.origin;
          person_hint_valid_ = true;
        }
        // Everything accumulated so far belongs to the object gesture.  From
        // this point onward, collect the Avatar's second gesture (the cleanup
        // destination) while waiting for the Clean_up! event.
        obs_objects_.clear();
        obs_pointings_.clear();
        point_again_count_ = 0;
        // The next valid gesture belongs to the destination phase. Do not let
        // the remembered target gesture leak into that selection.
        has_locked_pointing_ = false;
        has_last_valid_pointing_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "[ObserveTarget] Target confirmed without camera scan");
        changeStep(WaitForCleanCommand);
        break;
      }

      const double target_observe_timeout =
        (sub_step_ == 4) ? TIMEOUT_POINTING_WAIT :
        (sub_step_ == 7) ? 5.0 : TIMEOUT_OBSERVE;
      if (elapsed() >= target_observe_timeout) {
        stopBase();
        logObservationSummary("ObserveTarget");
        logPointingRayDiagnostics("ObserveTarget");
        // Interactive Cleanup requires the object indicated by the finger.
        // Do not silently substitute the highest-scoring YOLO box when no
        // stable finger/object match was obtained.
        RCLCPP_WARN(this->get_logger(),
                    "[ObserveTarget] No stable finger-to-object match in observation window");
        bool found = false;
        if (!found && point_again_count_ < MAX_POINT_AGAIN) {
          RCLCPP_WARN(this->get_logger(), "[ObserveTarget] Target not found, requesting re-point");
          point_again_count_++;
          sendMessage(MSG_POINT_IT_AGAIN);
          obs_objects_.clear();
          obs_pointings_.clear();
          has_locked_pointing_ = false;
          has_last_valid_pointing_ = false;
          changeStep(WaitForPickCommand);
        } else if (!found) {
          RCLCPP_WARN(this->get_logger(), "[ObserveTarget] No target after max re-points, giving up");
          giveUp();
          changeStep(WaitForResult);
        } else {
          changeStep(WaitForCleanCommand);
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForCleanCommand: {
      // As with Pick_it_up!, Unity normally plays the destination gesture
      // before sending Clean_up!. Keep the Avatar visible so that gesture is
      // buffered, then resolve it before allowing motion toward the object.
      reacquireAvatarForDestination();

      if (received_clean_command_) {
        RCLCPP_INFO(this->get_logger(),
                    "[WaitForClean] Got Clean_up! → using buffered destination gesture");
        received_clean_command_ = false;
        stopBase();
        headCenter();

        // In the official sequence the Avatar points to the destination
        // before sending Clean_up!, then immediately returns to its initial
        // position.  Resolve that buffered gesture now and go straight to the
        // already-known object instead of starting observation too late.
        if (selectDestinationByPointingRay()) {
          enableVision(false);
          changeStep(MoveToObject);
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "[WaitForClean] No buffered destination gesture; "
                      "entering recovery observation");
          has_locked_pointing_ = false;
          scan_index_ = 0;
          changeStep(ObserveDestination);
        }
      } else if (elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case ObserveDestination: {
      if (sub_step_ == 0) {
        if (lockRecentPointing()) {
          beginPointingHold("ObserveDest");
          sub_step_ = 2;
        } else {
          beginPointingWait("ObserveDest");
          sub_step_ = 4;
        }
      } else if (sub_step_ == 4) {
        reacquireAvatarForDestination();
        if (lockRecentPointing()) {
          RCLCPP_INFO(this->get_logger(),
                      "[ObserveDest] Pointing acquired; holding current view");
          state_enter_time_ = std::chrono::steady_clock::now();
          beginPointingHold("ObserveDest");
          sub_step_ = 2;
        }
      }

      const double destination_observe_timeout =
        (sub_step_ == 4) ? TIMEOUT_POINTING_WAIT : TIMEOUT_OBSERVE;
      if (elapsed() >= destination_observe_timeout) {
        stopBase();
        logObservationSummary("ObserveDest");
        bool found = selectDestinationByPointingRay();
        if (!found && point_again_count_ < MAX_POINT_AGAIN) {
          RCLCPP_WARN(this->get_logger(), "[ObserveDest] Dest not found, requesting re-point");
          point_again_count_++;
          sendMessage(MSG_POINT_IT_AGAIN);
          obs_objects_.clear();
          obs_pointings_.clear();
          changeStep(WaitForPickCommand);
        } else if (!found) {
          RCLCPP_WARN(this->get_logger(), "[ObserveDest] No destination, giving up");
          giveUp();
          changeStep(WaitForResult);
        } else {
          enableVision(false);
          changeStep(MoveToObject);
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case MoveToObject: {
      if (elapsed() > TIMEOUT_MOVE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0: {
          armObserve();
          headLookDown();
          gripperOpen();
          RCLCPP_INFO(this->get_logger(), "[MoveToObj] Preparing approach...");

          if (target_valid_) {
            const auto robot = getRobotPose();
            const double yaw = angleToPoint(target_position_.x, target_position_.y);
            double goal_x = target_position_.x;
            double goal_y = target_position_.y;
            if (robot.valid) {
              const double dx = target_position_.x - robot.x;
              const double dy = target_position_.y - robot.y;
              const double range = std::hypot(dx, dy);
              if (range > 1e-3) {
                goal_x -= OBJECT_OBSERVE_STOP_DIST * dx / range;
                goal_y -= OBJECT_OBSERVE_STOP_DIST * dy / range;
              }
            }
            sendNavGoal(goal_x, goal_y, yaw);
          }
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          // Navigate — Nav2 or direct drive
          if (target_valid_ && nav2_available_ && nav_goal_sent_) {
            if (nav_goal_reached_) {
              RCLCPP_INFO(this->get_logger(), "[MoveToObj] Nav2 goal reached");
              stopBase();
              sub_step_ = 3;
              enterTimedState();
            } else if (nav_goal_failed_) {
              RCLCPP_WARN(this->get_logger(), "[MoveToObj] Nav2 failed, fallback to direct");
              resetNavState();
              sub_step_ = 2;
              enterTimedState();
            }
          } else {
            sub_step_ = 2;
            enterTimedState();
          }
          break;
        }
        case 2: {
          // Direct drive toward target
          if (!target_valid_) {
            stopBase();
            RCLCPP_ERROR(this->get_logger(),
                         "[MoveToObj] No valid target position; refusing blind drive");
            giveUp();
            changeStep(WaitForResult);
            break;
          }

          double dist = distToPoint(target_position_.x, target_position_.y);
          double angle = angleToPoint(target_position_.x, target_position_.y);

          if (dist > OBJECT_OBSERVE_STOP_DIST) {
            if (std::abs(angle) > TURN_THRESHOLD) {
              double az = std::clamp(angle * 1.5, -0.5, 0.5);
              moveBase(0.05, 0.0, az);
            } else {
              double speed = std::min(0.2, dist * 0.4);
              moveBase(speed, 0.0, angle * 0.3);
            }
          } else {
            stopBase();
            RCLCPP_INFO(this->get_logger(), "[MoveToObj] Reached vicinity (%.2fm)", dist);
            sub_step_ = 3;
            enterTimedState();
          }
          break;
        }
        case 3: {
          // Start fine alignment once.  Re-sending the head trajectory on every
          // state-machine tick prevents the head from settling and floods the
          // vision gate with duplicate enable messages.
          enableVision(true);
          headLookAtFloor();
          has_new_objects_ = false;
          fine_target_seen_ = false;
          RCLCPP_INFO(this->get_logger(),
                      "[MoveToObj] Fine alignment on class='%s'",
                      target_class_.c_str());
          sub_step_ = 5;
          enterTimedState();
          break;
        }
        case 5: {
          // The coarse controller has already put the base at grasping range.
          // Only rotate to center the selected class; never drive blindly into
          // another object just because it is near the image centre.
          if (elapsed() < 1.0) break; // allow the head and detector to settle

          if (has_new_objects_) {
            has_new_objects_ = false;
            const DetectedObject *match = nullptr;
            double best_dist = 9999.0;
            for (const auto &o : latest_objects_.objects) {
              if (o.class_name != target_class_) continue;
              double d = std::hypot(o.bbox_cx - 320.0, o.bbox_cy - 240.0);
              if (d < best_dist) { best_dist = d; match = &o; }
            }

            if (match) {
              double dx = (match->bbox_cx - 320.0) / 320.0;
              if (match->has_3d_position) {
                // Replace the long-range estimate on the first close view,
                // then filter subsequent measurements to suppress depth
                // jitter without retaining the stale position.
                if (!fine_target_seen_) {
                  target_position_ = match->position_3d;
                  fine_target_seen_ = true;
                } else {
                  target_position_.x = 0.30 * target_position_.x +
                                       0.70 * match->position_3d.x;
                  target_position_.y = 0.30 * target_position_.y +
                                       0.70 * match->position_3d.y;
                  target_position_.z = 0.30 * target_position_.z +
                                       0.70 * match->position_3d.z;
                }
              }
              double target_dist = match->has_3d_position
                ? distToPoint(match->position_3d.x, match->position_3d.y)
                : OBJECT_OBSERVE_STOP_DIST;
              if (match->has_3d_position &&
                  target_dist < OBJECT_OBSERVE_MIN_DIST) {
                // Preserve camera visibility and physical clearance during
                // localization.  Final grasp approach happens only after the
                // target coordinate has been frozen.
                moveBase(-0.06, 0.0,
                         std::clamp(-dx * 0.35, -0.20, 0.20));
              } else if (match->has_3d_position &&
                         target_dist > OBJECT_OBSERVE_MAX_DIST) {
                moveBase(0.04, 0.0,
                         std::clamp(-dx * 0.35, -0.20, 0.20));
              } else if (std::abs(dx) <= VISUAL_SERVO_CENTER_TOLERANCE) {
                stopBase();
                RCLCPP_INFO(this->get_logger(),
                            "[MoveToObj] Grasp pose aligned: class='%s' "
                            "distance=%.2fm error=%.3f",
                            target_class_.c_str(), target_dist, dx);
                enableVision(false);
                sub_step_ = 4;
                break;
              } else {
                moveBase(0.0, 0.0,
                         std::clamp(-dx * 0.45, -0.25, 0.25));
              }
            } else {
              stopBase();
            }
          }

          if (elapsed() > VISUAL_SERVO_TIMEOUT) {
            stopBase();
            enableVision(false);
            if (!fine_target_seen_) {
              // The target was already selected from a stable multi-frame
              // pointing vote.  A floor object can disappear below the head
              // camera or behind the arm at close range; absence of a second
              // YOLO box must not invalidate that world-space coordinate.
              RCLCPP_WARN(this->get_logger(),
                          "[MoveToObj] Target not re-observed at close range; "
                          "continuing from stable pointed position "
                          "(%.2f, %.2f, %.2f)",
                          target_position_.x, target_position_.y,
                          target_position_.z);
              sub_step_ = 4;
            } else {
              RCLCPP_WARN(this->get_logger(),
                          "[MoveToObj] Fine alignment timed out, but close-range "
                          "target is valid at (%.2f, %.2f, %.2f)",
                          target_position_.x, target_position_.y,
                          target_position_.z);
              sub_step_ = 4;
            }
          }
          break;
        }
        case 4: {
          enableVision(false);
          changeStep(GraspObject);
          break;
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case GraspObject: {
      if (elapsed() > TIMEOUT_GRASP && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0:
          // Extend above the object while it is still safe, then align the
          // actual fingertip midpoint before lowering to floor height.  The
          // close-range target is frozen here: the arm can occlude the object
          // and corrupt a new depth estimate with its own surface.
          armGraspReady(); headLookAtFloor();
          enableVision(false);
          grasp_align_stable_cycles_ = 0;
          sub_step_ = 1; enterTimedState(); break;
        case 1:
          if (elapsed() > 2.0) { sub_step_ = 5; enterTimedState(); }
          break;
        case 5: {
          geometry_msgs::msg::Point gripper;
          const auto robot = getRobotPose();
          if (!target_valid_ || !robot.valid || !getGripperMidpoint(gripper)) {
            stopBase();
            enableVision(false);
            RCLCPP_ERROR(this->get_logger(),
                         "[Grasp] Fingertip alignment unavailable; "
                         "refusing blind grasp");
            giveUp();
            changeStep(WaitForResult);
            break;
          }

          const double world_dx = target_position_.x - gripper.x;
          const double world_dy = target_position_.y - gripper.y;
          const double error = std::hypot(world_dx, world_dy);
          // Convert the world error into base-frame forward/lateral motion.
          double forward = std::cos(robot.yaw) * world_dx +
                           std::sin(robot.yaw) * world_dy;
          double lateral = -std::sin(robot.yaw) * world_dx +
                            std::cos(robot.yaw) * world_dy;

          const bool aligned =
            std::abs(forward) <= GRIPPER_ALIGN_FORWARD_TOLERANCE &&
            std::abs(lateral) <= GRIPPER_ALIGN_LATERAL_TOLERANCE;
          // Once the selected target has been localized, TF provides a much
          // steadier relative measurement than close-range vision.  The arm
          // commonly occludes the object at this point, so do not require a
          // fresh YOLO box on every stable cycle.
          if (aligned) {
            grasp_align_stable_cycles_++;
          } else if (!aligned) {
            grasp_align_stable_cycles_ = 0;
          }

          if (grasp_align_stable_cycles_ >= GRIPPER_ALIGN_STABLE_CYCLES) {
            stopBase();
            enableVision(false);
            RCLCPP_INFO(this->get_logger(),
                        "[Grasp] Fingertip midpoint aligned forward=%.3fm "
                        "lateral=%.3fm total=%.3fm; lowering",
                        forward, lateral, error);
            armGraspLow();
            sub_step_ = 2;
            enterTimedState();
            break;
          }

          if (elapsed() >= GRIPPER_ALIGN_TIMEOUT) {
            stopBase();
            enableVision(false);
            RCLCPP_ERROR(this->get_logger(),
                         "[Grasp] Precise fingertip alignment failed: "
                         "forward=%.3fm lateral=%.3fm total=%.3fm; "
                         "refusing blind grasp",
                         forward, lateral, error);
            giveUp();
            changeStep(WaitForResult);
            break;
          }

          const double base_distance = distToPoint(
            target_position_.x, target_position_.y);
          if (base_distance <= APPROACH_MIN_SAFE_DIST && forward > 0.0) {
            forward = 0.0;
          }
          moveBase(std::clamp(forward * 0.45, -0.03, 0.03),
                   std::clamp(lateral * 0.45, -0.03, 0.03), 0.0);
          break;
        }
        case 2:
          if (elapsed() > 2.0) {
            geometry_msgs::msg::Point gripper;
            if (!getGripperMidpoint(gripper)) {
              stopBase();
              RCLCPP_ERROR(this->get_logger(),
                           "[Grasp] Cannot verify lowered fingertip height");
              giveUp();
              changeStep(WaitForResult);
              break;
            }
            const double height_above_target = gripper.z - target_position_.z;
            if (height_above_target > GRIPPER_MAX_HEIGHT_ABOVE_TARGET) {
              stopBase();
              RCLCPP_ERROR(this->get_logger(),
                           "[Grasp] Fingertips remain too high after lowering: "
                           "tip_z=%.3fm target_z=%.3fm difference=%.3fm",
                           gripper.z, target_position_.z, height_above_target);
              giveUp();
              changeStep(WaitForResult);
              break;
            }
            RCLCPP_INFO(this->get_logger(),
                        "[Grasp] Lowered fingertip height verified: "
                        "tip_z=%.3fm target_z=%.3fm; closing",
                        gripper.z, target_position_.z);
            gripperClose(); sub_step_ = 3; enterTimedState();
          }
          break;
        case 3:
          if (elapsed() > 3.0) { armCarry(); sub_step_ = 4; enterTimedState(); }
          break;
        case 4:
          if (elapsed() > 2.0) { changeStep(SendObjectGrasped); }
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

          if (!preparePlacementTarget()) {
            RCLCPP_ERROR(this->get_logger(),
                         "[MoveToDest] Cannot prepare placement target");
            giveUp();
            changeStep(WaitForResult);
            break;
          }

          if (dest_valid_) {
            // Nav2 must target an approach pose, never the furniture/bin
            // centre.  Direct drive below uses the same stand-off.
            const auto robot = getRobotPose();
            const double dx = robot.x - dest_position_.x;
            const double dy = robot.y - dest_position_.y;
            const double norm = std::hypot(dx, dy);
            const double ux = norm > 1e-6 ? dx / norm : 1.0;
            const double uy = norm > 1e-6 ? dy / norm : 0.0;
            const double goal_x =
              dest_position_.x + DESTINATION_COARSE_STOP_DIST * ux;
            const double goal_y =
              dest_position_.y + DESTINATION_COARSE_STOP_DIST * uy;
            const double yaw = std::atan2(
              dest_position_.y - goal_y, dest_position_.x - goal_x);
            sendNavGoal(goal_x, goal_y, yaw);
          }
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          if (dest_valid_ && nav2_available_ && nav_goal_sent_) {
            if (nav_goal_reached_) {
              stopBase();
              sub_step_ = 3;
              enterTimedState();
            } else if (nav_goal_failed_) {
              resetNavState();
              sub_step_ = 2;
              enterTimedState();
            }
          } else {
            sub_step_ = 2;
            enterTimedState();
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

          if (dist > DESTINATION_COARSE_STOP_DIST) {
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
        case 0: {
          if (!placement_ready_ && !preparePlacementTarget()) {
            RCLCPP_ERROR(this->get_logger(),
                         "[Release] Placement target unavailable");
            giveUp();
            changeStep(WaitForResult);
            break;
          }
          armReleaseReady(); headLookDown();
          release_align_stable_cycles_ = 0;
          release_height_adjustments_ = 0;
          sub_step_ = 1; enterTimedState(); break;
        }
        case 1:
          if (elapsed() > 2.0) { sub_step_ = 5; enterTimedState(); }
          break;
        case 5: {
          geometry_msgs::msg::Point gripper;
          const auto robot = getRobotPose();
          if (!robot.valid || !getGripperMidpoint(gripper)) {
            stopBase();
            RCLCPP_ERROR(this->get_logger(),
                         "[Release] Gripper alignment TF unavailable");
            giveUp();
            changeStep(WaitForResult);
            break;
          }

          const double world_dx = placement_position_.x - gripper.x;
          const double world_dy = placement_position_.y - gripper.y;
          double forward = std::cos(robot.yaw) * world_dx +
                           std::sin(robot.yaw) * world_dy;
          const double lateral = -std::sin(robot.yaw) * world_dx +
                                  std::cos(robot.yaw) * world_dy;
          const double error = std::hypot(world_dx, world_dy);
          const bool aligned =
            std::abs(forward) <= RELEASE_ALIGN_FORWARD_TOLERANCE &&
            std::abs(lateral) <= RELEASE_ALIGN_LATERAL_TOLERANCE;
          release_align_stable_cycles_ = aligned
            ? release_align_stable_cycles_ + 1 : 0;
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[Release] Aligning gripper=(%.2f,%.2f,%.2f) "
            "target=(%.2f,%.2f,%.2f) forward=%.3f lateral=%.3f",
            gripper.x, gripper.y, gripper.z,
            placement_position_.x, placement_position_.y,
            placement_position_.z, forward, lateral);

          if (release_align_stable_cycles_ >= RELEASE_ALIGN_STABLE_CYCLES) {
            stopBase();
            const double current_lift =
              getJointPosition("arm_lift_joint", ARM_RELEASE_READY[0]);
            const double desired_lift = std::clamp(
              current_lift + placement_position_.z - gripper.z, 0.0, 0.69);
            auto pose = ARM_RELEASE_READY;
            pose[0] = desired_lift;
            moveArm(pose, 1.5);
            release_height_adjustments_ = 1;
            RCLCPP_INFO(
              this->get_logger(),
              "[Release] XY aligned forward=%.3fm lateral=%.3fm total=%.3fm; "
              "height %.3f->%.3fm (lift=%.3f)",
              forward, lateral, error, gripper.z, placement_position_.z,
              desired_lift);
            sub_step_ = 2;
            enterTimedState();
            break;
          }

          if (elapsed() >= RELEASE_ALIGN_TIMEOUT) {
            stopBase();
            RCLCPP_ERROR(
              this->get_logger(),
              "[Release] Could not align gripper with placement point: "
              "forward=%.3fm lateral=%.3fm total=%.3fm",
              forward, lateral, error);
            giveUp();
            changeStep(WaitForResult);
            break;
          }

          const double base_distance = distToPoint(
            dest_position_.x, dest_position_.y);
          if (base_distance <= DESTINATION_MIN_BASE_CLEARANCE && forward > 0.0) {
            forward = 0.0;
          }
          moveBase(std::clamp(forward * 0.45, -0.035, 0.035),
                   std::clamp(lateral * 0.45, -0.035, 0.035), 0.0);
          break;
        }
        case 2: {
          if (elapsed() <= 2.0) break;
          geometry_msgs::msg::Point gripper;
          if (!getGripperMidpoint(gripper)) {
            RCLCPP_ERROR(this->get_logger(),
                         "[Release] Cannot verify release height");
            giveUp();
            changeStep(WaitForResult);
            break;
          }
          const double height_error = placement_position_.z - gripper.z;
          if (std::abs(height_error) > RELEASE_HEIGHT_TOLERANCE &&
              release_height_adjustments_ < 3) {
            const double current_lift =
              getJointPosition("arm_lift_joint", ARM_RELEASE_READY[0]);
            const double desired_lift = std::clamp(
              current_lift + height_error, 0.0, 0.69);
            auto pose = ARM_RELEASE_READY;
            pose[0] = desired_lift;
            moveArm(pose, 1.2);
            release_height_adjustments_++;
            RCLCPP_INFO(
              this->get_logger(),
              "[Release] Correcting height: actual=%.3fm target=%.3fm "
              "lift=%.3f attempt=%d",
              gripper.z, placement_position_.z, desired_lift,
              release_height_adjustments_);
            enterTimedState();
            break;
          }
          if (std::abs(height_error) > RELEASE_HEIGHT_TOLERANCE) {
            RCLCPP_ERROR(
              this->get_logger(),
              "[Release] Release height unreachable: actual=%.3fm "
              "target=%.3fm error=%.3fm",
              gripper.z, placement_position_.z, height_error);
            giveUp();
            changeStep(WaitForResult);
            break;
          }
          RCLCPP_INFO(this->get_logger(),
                      "[Release] Verified placement pose at "
                      "(%.2f, %.2f, %.2f); opening gripper",
                      gripper.x, gripper.y, gripper.z);
          gripperOpen();
          sub_step_ = 3;
          enterTimedState();
          break;
        }
        case 3:
          if (elapsed() > 3.0) {
            // Back away with the arm held still.  Retracting next to the
            // released object can sweep it out of the placement trigger.
            sub_step_ = 4; enterTimedState();
          }
          break;
        case 4:
          if (elapsed() < 1.5) {
            driveForward(-0.08);
          } else {
            stopBase();
            armInitial(); headCenter();
            sub_step_ = 6; enterTimedState();
          }
          break;
        case 6:
          if (elapsed() > 2.0) { changeStep(SendTaskFinished); }
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
      enableVision(false);
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
  int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
