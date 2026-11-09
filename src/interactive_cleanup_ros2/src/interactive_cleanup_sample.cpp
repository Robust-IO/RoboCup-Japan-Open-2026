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

#include "interactive_cleanup/place_destination_resolver.hpp"
#include "interactive_cleanup/gesture_capture_window.hpp"
#include "interactive_cleanup/pointing_geometry.hpp"
#include "interactive_cleanup/hsr_geometry.hpp"

#include <string>
#include <vector>
#include <map>
#include <set>
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
  friend struct CleanupTimingAccess;
public:
  explicit InteractiveCleanupSample(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : rclcpp::Node("interactive_cleanup_sample", options)
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

    sub_vision_ready_ = this->create_subscription<std_msgs::msg::Bool>(
      "/cleanup_vision/ready", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr msg) { vision_ready_ = msg->data; });

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
  // In recorded competition runs Pick_it_up! can arrive about 14 seconds
  // after the visible gesture has ended.  Keep only the current task's last
  // valid gesture (reset() clears it between tasks), long enough to bridge
  // that moderator delay.
  static constexpr double POINTING_MEMORY_SEC  =  20.0;
  static constexpr double POINT_ALIGN_TIMEOUT  =   2.0;
  static constexpr double POINT_ALIGN_ANGLE_THRESHOLD = 0.18;
  static constexpr double POINT_REALIGN_THRESHOLD     = 0.28;
  static constexpr double POINT_ALIGN_MAX_ANGULAR     = 0.45;
  static constexpr double AVATAR_TRACK_INTERVAL       = 0.25;
  static constexpr double AVATAR_TRACK_DEADBAND_RAD   = 0.045;
  static constexpr double AVATAR_OBSERVATION_MEMORY_SEC = 1.50;
  static constexpr double AVATAR_SEARCH_INTERVAL_SEC  = 2.00;
  static constexpr double AVATAR_SEARCH_MOTION_SEC    = 0.60;
  // Pose/hand inference in SIGVerse commonly produces one usable result only
  // every 2--3 seconds.  Do not move the head merely because that slow stream
  // has not refreshed inside the old two-second window.
  static constexpr double CLOSE_GESTURE_SEARCH_INTERVAL_SEC = 4.00;
  static constexpr double CLOSE_GESTURE_SEARCH_MOTION_SEC = 0.40;
  // Image-space destination rays are meaningful only after the head has
  // stopped moving.  Following the Avatar is mandatory, so discard samples
  // collected during a tracking correction and rebuild the short vote once
  // the camera has settled.
  static constexpr double DESTINATION_HEAD_SETTLE_SEC = 0.35;
  static constexpr int    MAX_POINT_ALIGN_RETRY = 0;

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
  // Vision reports the visible front surface of an object.  Merely aligning
  // the finger-trigger midpoint with that surface can leave a small object
  // just in front of the closing region.  Servo the open fingers a modest
  // distance around the surface, then use a tighter residual tolerance.
  static constexpr double GRIPPER_TARGET_INSERTION_DEPTH = 0.010;
  static constexpr double GRIPPER_ALIGN_FORWARD_TOLERANCE = 0.010;
  static double graspForwardTolerance(double target_height)
  {
    // An elevated target's 10 mm insertion must not be swallowed by a
    // matching 10 mm tolerance. Keep the successful floor-object window.
    return target_height < 0.15 ? GRIPPER_ALIGN_FORWARD_TOLERANCE : 0.003;
  }
  static constexpr double GRIPPER_ALIGN_LATERAL_TOLERANCE = 0.012;
  static bool graspHorizontalAligned(
    const geometry_msgs::msg::Point &target,
    const geometry_msgs::msg::Point &gripper, double yaw)
  {
    const double dx=target.x-gripper.x, dy=target.y-gripper.y;
    const double insertion=target.z < 0.15 ? 0.0 : GRIPPER_TARGET_INSERTION_DEPTH;
    const double forward=std::cos(yaw)*dx+std::sin(yaw)*dy+insertion;
    const double lateral=-std::sin(yaw)*dx+std::cos(yaw)*dy;
    return std::isfinite(forward) && std::isfinite(lateral) &&
      std::abs(forward) <= graspForwardTolerance(target.z) &&
      std::abs(lateral) <= GRIPPER_ALIGN_LATERAL_TOLERANCE;
  }
  static constexpr int    GRIPPER_ALIGN_STABLE_CYCLES = 3;
  static constexpr double GRIPPER_HEIGHT_TOLERANCE = 0.05;
  static constexpr double GRIPPER_DOWNWARD_BASE_HEIGHT = 0.066;
  static constexpr double GRIPPER_PREGRASP_CLEARANCE = 0.10;
  static constexpr double GRIPPER_CONTACT_HEIGHT_OFFSET = -0.015;
  static constexpr double ARM_LIFT_MAX = 0.69;
  static constexpr double TARGET_MIN_GRASP_HEIGHT = -0.03;
  static constexpr double TARGET_MAX_GRASP_HEIGHT = 0.72;
  // Destination motion has two distinct stages.  The base first stops where
  // carried-object collisions are unlikely, then the real finger collision
  // midpoint is servoed onto a point inside Unity's placement trigger.
  static constexpr double DESTINATION_COARSE_STOP_DIST = 0.78;
  static constexpr double DESTINATION_MIN_BASE_CLEARANCE = 0.48;
  static constexpr double RELEASE_ALIGN_TIMEOUT = 20.0;
  // The official surface triggers are much wider than this tolerance. A
  // 3 cm hard threshold caused a deadlock at 3.5 cm when the base-clearance
  // guard correctly refused to drive farther into the table.
  static constexpr double RELEASE_ALIGN_FORWARD_TOLERANCE = 0.040;
  static constexpr double RELEASE_ALIGN_LATERAL_TOLERANCE = 0.030;
  static constexpr int    RELEASE_ALIGN_STABLE_CYCLES = 3;
  static constexpr double RELEASE_HEIGHT_TOLERANCE = 0.060;
  static constexpr double TURN_THRESHOLD       = 0.15;
  static constexpr double VISUAL_SERVO_TIMEOUT = 12.0;
  static constexpr double VISUAL_SERVO_CENTER_TOLERANCE = 0.10;
  static constexpr double TARGET_EARLY_CONFIRM_DELAY = 1.2;
  static constexpr int    STABLE_TARGET_MIN_FRAMES = 6;
  // The vision node intentionally publishes the dedicated 13-class model
  // down to 0.25 confidence.  A second 0.45 gate here silently discarded
  // repeatable detections (the failed run saw bear_doll in all 30 frames).
  // Temporal persistence and ray distance provide the stronger safeguards.
  static constexpr double STABLE_TARGET_MIN_CONFIDENCE = 0.25;
  static constexpr double STABLE_TARGET_MAX_RAY_2D_PX = 100.0;
  // Persistence cannot turn a geometrically unrelated object into the
  // pointed target. Correct picks in recorded runs are normally within
  // 0.08--0.22 m; the old 1.25 m gate admitted rabbit_doll at 1.12 m.
  static constexpr double STABLE_TARGET_MAX_BEST_RAY_3D_M = 0.45;
  static constexpr double STABLE_TARGET_MAX_AVG_RAY_3D_M = 0.55;
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
    // Move the release point from the trigger/container centre toward the
    // robot. This keeps the fingers inside the destination while preserving
    // base clearance at narrow containers and furniture edges.
    double approach_inset;
    double center_offset_x;
    double center_offset_y;
    bool drop_in;
  };

  // =========================================================================
  // State variables
  // =========================================================================
  int step_;
  bool is_started_  = false;
  bool vision_ready_ = false; // Process readiness survives task reset.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_vision_ready_;
  bool is_finished_ = false;
  bool is_failed_   = false;
  std::string failed_detail_;
  bool received_pick_command_  = false;
  bool received_clean_command_ = false;
  size_t clean_continuation_frame_count_ = 0;
  bool early_pick_attempted_ = false;
  bool early_pick_observing_ = false;
  bool preconfirmed_target_ = false;
  std::chrono::steady_clock::time_point preconfirmed_target_time_;
  std::chrono::steady_clock::time_point clean_drain_deadline_;
  // Same-host RGB ingress and protocol receipt share CLOCK_MONOTONIC.
  // Source timestamps remain exclusively responsible for sensor geometry.
  static int64_t ingressNowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  interactive_cleanup::GestureCaptureWindow destination_window_;
  // Source stamps are for TF/RGB-D only. Motion and protocol admission use
  // the shared host monotonic clock, unaffected by host wall-clock steps.
  rclcpp::Clock perception_stamp_clock_{RCL_SYSTEM_TIME}; // Log throttling only.
  int64_t head_observation_not_before_ns_ = 0; // RGB ingress, CLOCK_MONOTONIC.
  std::chrono::steady_clock::time_point head_motion_ready_time_{};

  bool receivedAfterHeadMotion(int64_t ingress_ns) const
  {
    return ingress_ns > 0 && ingress_ns >= head_observation_not_before_ns_ &&
      std::chrono::steady_clock::now() >= head_motion_ready_time_;
  }
  bool received_yes_ = false;
  bool received_no_  = false;

  std::chrono::steady_clock::time_point state_enter_time_;
  bool state_timer_initialized_ = false;
  int sub_step_ = 0;
  int scan_index_ = 0;
  int point_align_retry_count_ = 0;
  bool ready_ack_sent_ = false;
  std::chrono::steady_clock::time_point last_ready_ack_time_;
  std::chrono::steady_clock::time_point last_pointing_recv_time_;
  std::chrono::steady_clock::time_point last_valid_pointing_time_;
  std::chrono::steady_clock::time_point observe_align_start_time_;
  std::chrono::steady_clock::time_point observe_scan_start_time_;
  std::chrono::steady_clock::time_point last_avatar_track_time_;
  std::chrono::steady_clock::time_point last_avatar_search_time_;
  std::chrono::steady_clock::time_point last_person_observation_time_;
  std::chrono::steady_clock::time_point last_raw_pointing_time_;
  std::chrono::steady_clock::time_point last_close_gesture_search_time_;
  std::chrono::steady_clock::time_point pointing_camera_ready_time_;
  std::chrono::steady_clock::time_point destination_camera_ready_time_;
  int avatar_search_pose_index_ = 0;
  int close_gesture_search_index_ = 0;
  bool close_gesture_search_active_ = false;
  double close_gesture_anchor_pan_ = 0.0;
  double close_gesture_anchor_tilt_ = 0.0;
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
  PointingDirection   retained_arm_pointing_;
  bool has_retained_arm_pointing_ = false;
  bool locked_historical_arm_ = false;
  std::chrono::steady_clock::time_point retained_arm_time_{};
  bool has_new_objects_  = false;
  bool has_new_pointing_ = false;
  bool has_last_valid_pointing_ = false;
  bool has_locked_pointing_ = false;
  // World-space hypotheses from the pickup gesture survive view changes.
  // A head-aim ray is provisional until object evidence supports it.
  std::vector<PointingDirection> pickup_ray_hypotheses_;
  std::chrono::steady_clock::time_point last_person_seen_time_{};
  bool has_person_observation_ = false;
  double last_person_pixel_x_ = 320.0;
  double last_person_pixel_y_ = 240.0;

  // Accumulated observations for voting
  std::vector<DetectedObjectArray> obs_objects_;
  std::vector<PointingDirection>   obs_pointings_;
  std::vector<PointingDirection>   pickup_pointing_history_;
  // The place cue must never share the target observation buffer. Target
  // confirmation is the protocol boundary: after the return-to-person head
  // motion settles, every valid ray belongs to the destination phase until
  // the Clean_up! callback closes it.
  std::vector<PointingDirection>   destination_pointing_history_;
  // World rays are already transformed into odom at capture time and remain
  // valid across head tracking motions. Image rays are camera-pose dependent
  // and therefore use a separate epoch that is reset after every correction.
  std::vector<PointingDirection>   destination_image_pointing_history_;
  // Clean_up! closes the destination capture window at callback time so
  // frames arriving before the state-machine timer runs cannot cross phases.
  bool destination_capture_closed_ = false;
  bool destination_gesture_started_ = false;

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
        if (vision_ready_) sendReadyAck("initial handshake");
        else RCLCPP_INFO(this->get_logger(), "Waiting for vision inference readiness before I_am_ready");
      } else if (step_ == WaitForPickCommand) {
        sendReadyAck("moderator retried handshake");
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring unexpected Are_you_ready? while in [%s]",
          stepName(step_).c_str());
      }
    }
    else if (message == MSG_PICK_IT_UP) {
      if (step_ == WaitForPickCommand ||
          (step_ == ObserveTarget && early_pick_observing_)) {
        received_pick_command_ = true;
        destination_window_.open(ingressNowNs());
        if (early_pick_observing_) {
          early_pick_observing_ = false;
          state_enter_time_ = std::chrono::steady_clock::now();
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring out-of-phase Pick_it_up! while in [%s]",
          stepName(step_).c_str());
      }
    }
    else if (message == MSG_CLEAN_UP) {
      // Clean_up! closes destination capture. A short drain may retain only
      // frames that continue the final pre-command world ray; it cannot start
      // a new gesture or change the selected direction.
      if (step_ == ObserveTarget || step_ == WaitForCleanCommand) {
        if (!received_clean_command_) {
          destination_window_.close(ingressNowNs());
          clean_drain_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::seconds(2);
          clean_continuation_frame_count_ = 0;
        }
        received_clean_command_ = true;
        if (step_ == WaitForCleanCommand) {
          destination_capture_closed_ = true;
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Ignoring out-of-phase Clean_up! while in [%s]",
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

  void objectsCallback(const DetectedObjectArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    has_new_objects_ = true;
    if ((step_ == ObserveTarget || step_ == WaitForCleanCommand) &&
        receivedAfterHeadMotion(msg->received_steady_ns))
      obs_objects_.push_back(*msg);
  }

  bool continuesFinalPreCleanRay(const PointingDirection &candidate) const
  {
    constexpr int64_t MAX_PRE_CLEAN_RAY_AGE_NS = 500000000LL;
    constexpr double MAX_CONTINUATION_ANGLE_RAD = 12.0 * M_PI / 180.0;
    if (!received_clean_command_ || destination_window_.end_ns <= 0 ||
        std::chrono::steady_clock::now() >= clean_drain_deadline_ ||
        candidate.received_steady_ns <= destination_window_.end_ns ||
        !candidate.is_valid || !candidate.person_is_valid ||
        !candidate.depth_direction_valid || !candidate.plane_valid) return false;

    const PointingDirection *anchor = nullptr;
    for (const auto &pointing : destination_pointing_history_) {
      if (pointing.received_steady_ns <= destination_window_.end_ns &&
          pointing.is_valid && pointing.person_is_valid &&
          pointing.depth_direction_valid && pointing.plane_valid &&
          (anchor == nullptr || pointing.received_steady_ns >
            anchor->received_steady_ns)) {
        anchor = &pointing;
      }
    }
    if (anchor == nullptr ||
        destination_window_.end_ns - anchor->received_steady_ns < 0 ||
        destination_window_.end_ns - anchor->received_steady_ns >
          MAX_PRE_CLEAN_RAY_AGE_NS) return false;

    const auto &a = anchor->direction;
    const auto &b = candidate.direction;
    const double norm_a = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    const double norm_b = std::sqrt(b.x*b.x + b.y*b.y + b.z*b.z);
    if (!std::isfinite(norm_a) || !std::isfinite(norm_b) ||
        norm_a < 1e-6 || norm_b < 1e-6) return false;
    const double dot = (a.x*b.x + a.y*b.y + a.z*b.z) / (norm_a*norm_b);
    return std::isfinite(dot) &&
      dot >= std::cos(MAX_CONTINUATION_ANGLE_RAD);
  }

  void pointingCallback(const PointingDirection::SharedPtr msg)
  {
    latest_pointing_ = *msg;
    has_new_pointing_ = true;
    last_pointing_recv_time_ = std::chrono::steady_clock::now();

    // Visibility can suppress a blind scan even while geometry is unsettled.
    // Do not use these pixels for tracking until the existing motion gate passes.
    if (msg->person_is_valid && rclcpp::Time(msg->header.stamp).nanoseconds() > 0)
      last_person_seen_time_ = last_pointing_recv_time_;

    // Pose inference can miss isolated frames while the Avatar is plainly
    // visible. Keep the most recent positive torso observation instead of
    // letting one negative frame immediately restart the head search.
    if (msg->person_is_valid && receivedAfterHeadMotion(msg->received_steady_ns)) {
      has_person_observation_ = true;
      last_person_pixel_x_ = msg->person_pixel_x;
      last_person_pixel_y_ = msg->person_pixel_y;
      // Keep the arm/fingertip in view even when depth or TF is unavailable.
      // Use the image gesture from this same observation, not a stale ray.
      if (msg->is_valid) {
        last_person_pixel_x_ = interactive_cleanup::gestureFramingCenter(
          msg->person_pixel_x, msg->wrist_pixel_x, msg->point_pixel_x);
        last_person_pixel_y_ = interactive_cleanup::gestureFramingCenter(
          msg->person_pixel_y, msg->wrist_pixel_y, msg->point_pixel_y);
      }
      last_person_observation_time_ = last_pointing_recv_time_;
    }
    // A segment that the lock path already rejects must not postpone the
    // local search for another four seconds on every crop detection.
    const double raw_segment = std::hypot(
      msg->point_pixel_x-msg->wrist_pixel_x, msg->point_pixel_y-msg->wrist_pixel_y);
    if (msg->is_valid && (msg->depth_direction_valid ||
        (std::isfinite(raw_segment) && raw_segment >= MIN_PICKUP_IMAGE_SEGMENT))) {
      last_raw_pointing_time_ = last_pointing_recv_time_;
    }

    // Keep the recovery hint attached to the moving Avatar.  The original
    // pickup ray can be metres behind the person by the time the destination
    // gesture begins, so it must not remain the only world-space hint.
    if (step_ == WaitForCleanCommand && msg->is_valid && msg->person_is_valid) {
      person_hint_position_ = msg->origin;
      person_hint_valid_ = true;
    }

    const bool pickup_phase =
      step_ == WaitForPickCommand || step_ == ObserveTarget;
    // An odom ray survives head movement. Keep full-arm evidence separate
    // from cropped fingers; a later crop must not erase the first gesture.
    // After Pick, only inference whose RGB entered this host before Pick may update it.
    const auto capture_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    const bool before_pick = destination_window_.begin_ns == 0 ||
      (msg->received_steady_ns > 0 && msg->received_steady_ns <= destination_window_.begin_ns);
    // Pick_it_up! can arrive before the Avatar has formed the target gesture.
    // Keep accepting target-ray samples through the Pick..Clean_up interval;
    // the cutoff prevents delayed destination-gesture frames from entering
    // the pickup history after Clean_up! closes that interval.
    const bool inside_target_capture_window = destination_window_.end_ns <= 0 ||
      (msg->received_steady_ns > 0 &&
       msg->received_steady_ns <= destination_window_.end_ns);
    if (pickup_phase && msg->is_valid &&
        msg->arm_ray_valid && capture_ns > 0 && before_pick &&
        (!has_retained_arm_pointing_ || capture_ns >
          rclcpp::Time(retained_arm_pointing_.header.stamp).nanoseconds())) {
      const auto &o = msg->arm_ray_origin;
      const auto &d = msg->arm_ray_direction;
      const double norm = std::hypot(std::hypot(d.x, d.y), d.z);
      if (std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) &&
          std::isfinite(norm) && norm > 1e-6) {
        retained_arm_pointing_ = *msg;
        retained_arm_pointing_.origin = o;
        retained_arm_pointing_.direction.x = d.x/norm;
        retained_arm_pointing_.direction.y = d.y/norm;
        retained_arm_pointing_.direction.z = d.z/norm;
        retained_arm_time_ = last_pointing_recv_time_;
        has_retained_arm_pointing_ = true;
      }
    }
    const bool pickup_camera_settled =
      receivedAfterHeadMotion(msg->received_steady_ns) &&
      (pointing_camera_ready_time_.time_since_epoch().count() == 0 ||
       last_pointing_recv_time_ >= pointing_camera_ready_time_);
    // Preserve the original successful behavior: a valid pointing ray may be
    // off-centre because it is precisely what tells the head where to look.
    // The only hard temporal requirement is that the camera has stopped.
    const bool pickup_has_recent_person =
      msg->person_is_valid || hasRecentVisibleAvatar();
    const bool accept_valid_gesture =
      msg->is_valid &&
      (!pickup_phase ||
       (pickup_camera_settled && pickup_has_recent_person));
    if (pickup_phase) RCLCPP_INFO_THROTTLE(this->get_logger(), perception_stamp_clock_, 2000,
      "[GestureAudit] state=%s valid=%d person=%d settled=%d accepted=%d "
      "depth=%d confidence=%.3f history=%zu locked=%d",
      stepName(step_).c_str(), msg->is_valid, pickup_has_recent_person,
      pickup_camera_settled, accept_valid_gesture, msg->depth_direction_valid,
      msg->confidence, pickup_pointing_history_.size(), has_locked_pointing_);
    if (accept_valid_gesture) {
      if (pickup_phase && inside_target_capture_window) {
        pickup_pointing_history_.push_back(*msg);
        if (pickup_pointing_history_.size() > 8) {
          pickup_pointing_history_.erase(pickup_pointing_history_.begin());
        }
      }
      last_valid_pointing_ = *msg;
      has_last_valid_pointing_ = true;
      last_valid_pointing_time_ = last_pointing_recv_time_;
      if (pickup_phase && before_pick) {
        PointingDirection hypothesis;
        if (buildCurrentPointing(hypothesis) &&
            (pickup_ray_hypotheses_.empty() ||
             pickup_ray_hypotheses_.back().header.stamp != hypothesis.header.stamp)) {
          pickup_ray_hypotheses_.push_back(hypothesis);
          if (pickup_ray_hypotheses_.size() > 60)
            pickup_ray_hypotheses_.erase(pickup_ray_hypotheses_.begin());
        }
      }
    }
    if (step_ == ObserveTarget || step_ == WaitForCleanCommand)
      obs_pointings_.push_back(*msg);
    const bool destination_camera_settled =
      receivedAfterHeadMotion(msg->received_steady_ns) &&
      (destination_camera_ready_time_.time_since_epoch().count() == 0 ||
       last_pointing_recv_time_ >= destination_camera_ready_time_);
    const bool in_destination_capture_window =
      destination_window_.accepts(msg->received_steady_ns);
    const bool clean_ray_continuation =
      !in_destination_capture_window && continuesFinalPreCleanRay(*msg);
    if (accept_valid_gesture &&
        (in_destination_capture_window || clean_ray_continuation) &&
        step_ == WaitForCleanCommand &&
        (destination_camera_settled || msg->plane_valid)) {
      if (clean_ray_continuation && clean_continuation_frame_count_ == 0) {
        RCLCPP_INFO(this->get_logger(),
          "[WaitForClean] Draining frames that continue the final pre-command world ray");
      }
      if (!destination_gesture_started_) {
        destination_gesture_started_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "[WaitForClean] Destination gesture captured in settled tracked view");
      }
      destination_pointing_history_.push_back(*msg);
      if (destination_camera_settled && !clean_ray_continuation) {
        destination_image_pointing_history_.push_back(*msg);
      }
      if (clean_ray_continuation) ++clean_continuation_frame_count_;
      constexpr size_t MAX_DESTINATION_POINTING_HISTORY = 60;
      if (destination_pointing_history_.size() > MAX_DESTINATION_POINTING_HISTORY) {
        destination_pointing_history_.erase(destination_pointing_history_.begin());
      }
      if (destination_image_pointing_history_.size() >
          MAX_DESTINATION_POINTING_HISTORY) {
        destination_image_pointing_history_.erase(
          destination_image_pointing_history_.begin());
      }
    }
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
    if (message == MSG_TASK_FINISHED || message == MSG_OBJECT_GRASPED) {
      RCLCPP_INFO(this->get_logger(),
        "[ProtocolAudit] Local publish completed for '%s'; moderator-topic "
        "subscribers=%zu. This is not a moderator acknowledgement.",
        message.c_str(), pub_msg_->get_subscription_count());
    }
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
  std::vector<double> downwardGraspPoseForTipHeight(double desired_tip_z) const
  {
    auto pose = ARM_GRASP_LOW;
    pose[0] = std::clamp(
      desired_tip_z - GRIPPER_DOWNWARD_BASE_HEIGHT, 0.0, ARM_LIFT_MAX);
    return pose;
  }
  void armGraspReadyForTarget()
  {
    moveArm(downwardGraspPoseForTipHeight(
      target_position_.z + GRIPPER_PREGRASP_CLEARANCE), 1.5);
  }
  void armGraspAtTargetHeight()
  {
    moveArm(downwardGraspPoseForTipHeight(
      target_position_.z + GRIPPER_CONTACT_HEIGHT_OFFSET), 1.5);
  }
  void armCarry()        { moveArm(ARM_CARRY, 1.5); }
  void armReleaseReady() { moveArm(ARM_RELEASE_READY, 1.5); }

  // =========================================================================
  // Head control
  // =========================================================================
  void moveHead(double pan, double tilt, double dur)
  {
    const auto motion = interactive_cleanup::planHeadMotion(
      getJointPosition("head_pan_joint"), getJointPosition("head_tilt_joint"),
      pan, tilt, dur);
    pan = motion.pan; tilt = motion.tilt; dur = motion.duration;
    // Use the limit-adjusted duration for both execution and observation.
    // RGB ingress before this deadline remains ineligible even if inference
    // finishes later. This is a local receipt barrier, not capture-time proof.
    head_motion_ready_time_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dur + DESTINATION_HEAD_SETTLE_SEC));
    head_observation_not_before_ns_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        head_motion_ready_time_.time_since_epoch()).count();
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

  void headLookAtTarget()
  {
    // Approximate the head optical centre in odom.  The base has already
    // turned toward the remembered target, so only a height-dependent tilt
    // is needed here.  Floor targets still reach the old -1.05 limit, while
    // table targets remain in view for close-range verification.
    constexpr double camera_height = 1.20;
    const double horizontal_range = std::max(
      0.20, distToPoint(target_position_.x, target_position_.y));
    const double tilt = std::clamp(
      std::atan2(target_position_.z - camera_height, horizontal_range),
      -1.05, 0.20);
    moveHead(0.0, tilt, 1.0);
    RCLCPP_INFO(this->get_logger(),
                "[MoveToObj] Looking at remembered target height z=%.2fm tilt=%.2f",
                target_position_.z, tilt);
  }

  bool aimHeadAlongImagePointing(const PointingDirection &pointing)
  {
    // Preserve the field-tested task-1 behavior: after Pick_it_up!, move once
    // to where the visible forearm/finger line exits the current image, then
    // collect a fresh multi-frame object vote in that directed view.
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

    edge_scale = std::clamp(edge_scale + 0.35, 0.5, 10.0);

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
      current_pan + std::clamp(-image_yaw, -0.70, 0.70), interactive_cleanup::HSR_HEAD_PAN_MIN, interactive_cleanup::HSR_HEAD_PAN_MAX);
    const double target_tilt = std::clamp(
      current_tilt + std::clamp(-image_pitch, -0.55, 0.55), interactive_cleanup::HSR_HEAD_TILT_MIN, interactive_cleanup::HSR_HEAD_TILT_MAX);
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

  static constexpr double MIN_PICKUP_IMAGE_SEGMENT = 8.0;

  bool aimHeadAlongPointing(const PointingDirection &pointing)
  {
    if (locked_historical_arm_) {
      // Retained image pixels belong to an earlier head pose. Start voting
      // in the current view rather than applying that stale pixel offset.
      RCLCPP_INFO(this->get_logger(),
        "[PickupMemory] Holding current view for retained world-ray matching");
      return false;
    }
    const double image_length = std::hypot(
      pointing.point_pixel_x - pointing.wrist_pixel_x,
      pointing.point_pixel_y - pointing.wrist_pixel_y);
    if (image_length >= MIN_PICKUP_IMAGE_SEGMENT) {
      return aimHeadAlongImagePointing(pointing);
    }

    // A frontal gesture can collapse to a tiny image segment.  It is usable
    // only when synchronized RGB-D measured the 3-D direction; pose/image
    // fallback alone produced the upward false ray in the 22:48 task 2.
    if (!pointing.depth_direction_valid ||
        pointing.depth_direction_samples < 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "[ObserveTarget] Rejecting short image gesture without measured "
        "RGB-D direction");
      return false;
    }

    const auto robot = getRobotPose();
    if (!robot.valid) return false;
    constexpr double projection_distance = 2.5;
    constexpr double camera_height = 1.20;
    const double look_x =
      pointing.origin.x + projection_distance * pointing.direction.x;
    const double look_y =
      pointing.origin.y + projection_distance * pointing.direction.y;
    const double look_z =
      pointing.origin.z + projection_distance * pointing.direction.z;
    const double horizontal = std::max(
      0.20, std::hypot(look_x - robot.x, look_y - robot.y));
    const double target_pan = std::clamp(
      normalizeAngle(std::atan2(look_y - robot.y, look_x - robot.x) -
                     robot.yaw),
      interactive_cleanup::HSR_HEAD_PAN_MIN, interactive_cleanup::HSR_HEAD_PAN_MAX);
    const double target_tilt = std::clamp(
      std::atan2(look_z - camera_height, horizontal), interactive_cleanup::HSR_HEAD_TILT_MIN, interactive_cleanup::HSR_HEAD_TILT_MAX);
    target_aim_origin_pan_ = getJointPosition("head_pan_joint");
    target_aim_origin_tilt_ = getJointPosition("head_tilt_joint");
    target_aim_pan_ = target_pan;
    target_aim_tilt_ = target_tilt;
    target_aim_pose_valid_ = true;
    moveHead(target_pan, target_tilt, 0.8);
    RCLCPP_INFO(
      this->get_logger(),
      "[ObserveTarget] Frontal gesture aimed from measured RGB-D ray: "
      "pan=%.2f tilt=%.2f", target_pan, target_tilt);
    return true;
  }

  void headScanStep(const std::vector<HeadPose> &pattern, int idx)
  {
    int i = idx % static_cast<int>(pattern.size());
    moveHead(pattern[i].pan, pattern[i].tilt, 1.0);
  }

  void invalidatePickupPointingForHeadMotion(
    const std::chrono::steady_clock::time_point &now_time,
    double motion_sec)
  {
    if (step_ != WaitForPickCommand &&
        !(step_ == ObserveTarget && !has_locked_pointing_)) {
      return;
    }
    // A low depth-fit error does not establish that the arm direction is
    // correct. Reacquire in a settled view instead of locking a transient
    // estimate and substituting an approximate world-space head aim.
    has_last_valid_pointing_ = false;
    pickup_pointing_history_.clear();
    obs_pointings_.clear();
    pointing_camera_ready_time_ = now_time +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(
          motion_sec + DESTINATION_HEAD_SETTLE_SEC));
  }

  bool trackAvatarWithHead()
  {
    if (!hasRecentVisibleAvatar()) return false;

    const auto now_time = std::chrono::steady_clock::now();
    const bool collecting_pickup = step_ == WaitForPickCommand ||
      (step_ == ObserveTarget && !has_locked_pointing_);
    if (collecting_pickup && close_gesture_search_active_ &&
        last_close_gesture_search_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now_time-last_close_gesture_search_time_).count() <
          CLOSE_GESTURE_SEARCH_INTERVAL_SEC && !personOutsideGestureCaptureEnvelope()) {
      // Let the local search actually observe its chosen view. Otherwise
      // torso tracking immediately centres it again before the next sample.
      return false;
    }
    if (collecting_pickup &&
        pointing_camera_ready_time_.time_since_epoch().count() != 0) {
      // Do not restart a 0.65 s settling timer every 0.25 s. Give inference
      // a settled epoch to collect multiple frames, unless the person is
      // about to leave the image. Even then finish the current head motion.
      if (now_time < pointing_camera_ready_time_ ||
          (now_time < pointing_camera_ready_time_ + std::chrono::milliseconds(700) &&
           !personOutsideGestureCaptureEnvelope())) return false;
    }
    // Never apply the same stale pixel error repeatedly after commanding a
    // head correction. Wait for a new positive person observation first.
    if (last_avatar_track_time_.time_since_epoch().count() != 0 &&
        last_person_observation_time_ <= last_avatar_track_time_) {
      return false;
    }
    if (last_avatar_track_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now_time - last_avatar_track_time_).count() <
          AVATAR_TRACK_INTERVAL) {
      return false;
    }

    // Intrinsics reported by the current scene: fx=fy=554, cx=320, cy=240.
    // Positive image x is right while positive head pan is left. Negative
    // head tilt looks down, matching positive image y.
    constexpr double fx = 554.0, fy = 554.0, cx = 320.0, cy = 240.0;
    const double image_yaw = std::atan2(last_person_pixel_x_ - cx, fx);
    const double image_pitch = std::atan2(last_person_pixel_y_ - cy, fy);
    if (std::abs(image_yaw) <= AVATAR_TRACK_DEADBAND_RAD &&
        std::abs(image_pitch) <= AVATAR_TRACK_DEADBAND_RAD) {
      return false;
    }
    const double current_pan = getJointPosition("head_pan_joint");
    const double current_tilt = getJointPosition("head_tilt_joint");
    const double target_pan = std::clamp(current_pan - image_yaw, interactive_cleanup::HSR_HEAD_PAN_MIN, interactive_cleanup::HSR_HEAD_PAN_MAX);
    const double target_tilt = std::clamp(current_tilt - image_pitch, interactive_cleanup::HSR_HEAD_TILT_MIN, interactive_cleanup::HSR_HEAD_TILT_MAX);
    if (std::abs(target_pan - current_pan) < 0.01 &&
        std::abs(target_tilt - current_tilt) < 0.01) {
      return false;
    }
    moveHead(target_pan, target_tilt, 0.3);
    invalidatePickupPointingForHeadMotion(now_time, 0.3);
    last_avatar_track_time_ = now_time;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[AvatarTrack] person_pixel=(%.0f,%.0f) head=(%.2f,%.2f)->(%.2f,%.2f)",
      last_person_pixel_x_, last_person_pixel_y_,
      current_pan, current_tilt, target_pan, target_tilt);
    if (step_ == WaitForCleanCommand) {
      // Pixel rays collected before this correction belong to a different
      // camera pose.  Never project them through the new camera transform.
      destination_image_pointing_history_.clear();
      destination_camera_ready_time_ = now_time +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(0.3 + DESTINATION_HEAD_SETTLE_SEC));
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[WaitForClean] Following moving Avatar; resetting destination image samples");
    }
    return true;
  }

  bool hasRecentVisibleAvatar(
    double max_age_sec = AVATAR_OBSERVATION_MEMORY_SEC) const
  {
    if (!has_person_observation_) return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      last_person_observation_time_).count();
    return age <= max_age_sec;
  }

  bool hasRecentRawPointing(double max_age_sec = 4.0) const
  {
    if (last_raw_pointing_time_.time_since_epoch().count() == 0) return false;
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_raw_pointing_time_).count() <=
      max_age_sec;
  }

  bool personOutsideGestureCaptureEnvelope() const
  {
    if (!hasRecentVisibleAvatar()) return false;
    // Hold a settled camera epoch while the complete gesture remains safely
    // framed, but never let that hold lose a person who walks toward an edge.
    return last_person_pixel_x_ < 96.0 || last_person_pixel_x_ > 544.0 ||
           last_person_pixel_y_ < 72.0 || last_person_pixel_y_ > 408.0;
  }

  void searchGestureAroundVisibleAvatar()
  {
    const auto now_time = std::chrono::steady_clock::now();
    if ((step_ == WaitForPickCommand || step_ == ObserveTarget) &&
        now_time < pointing_camera_ready_time_ + std::chrono::milliseconds(700)) return;
    if (now_time < pointing_camera_ready_time_ ||
        (last_avatar_track_time_.time_since_epoch().count() != 0 &&
         std::chrono::duration<double>(now_time-last_avatar_track_time_).count() <
           AVATAR_SEARCH_INTERVAL_SEC)) return;
    if (last_close_gesture_search_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(
          now_time - last_close_gesture_search_time_).count() <
          CLOSE_GESTURE_SEARCH_INTERVAL_SEC) {
      return;
    }
    if (!close_gesture_search_active_) {
      close_gesture_search_index_ = 0;
      close_gesture_search_active_ = true;
    }
    // Recompute the sweep centre from the latest Avatar/hand observation on
    // every step. The recovery search therefore keeps following a moving
    // nearby person rather than scanning around a stale head pose.
    constexpr double fx = 554.0, fy = 554.0, cx = 320.0, cy = 240.0;
    const double current_pan = getJointPosition("head_pan_joint");
    const double current_tilt = getJointPosition("head_tilt_joint");
    const double image_yaw = std::atan2(last_person_pixel_x_ - cx, fx);
    const double image_pitch = std::atan2(last_person_pixel_y_ - cy, fy);
    close_gesture_anchor_pan_ = std::clamp(
      current_pan - image_yaw, interactive_cleanup::HSR_HEAD_PAN_MIN, interactive_cleanup::HSR_HEAD_PAN_MAX);
    close_gesture_anchor_tilt_ = std::clamp(
      current_tilt - image_pitch, interactive_cleanup::HSR_HEAD_TILT_MIN, interactive_cleanup::HSR_HEAD_TILT_MAX);
    // A close Avatar can fill the image so PoseLandmarker sees the torso but
    // the pointing hand lies just outside the frame. Cover adjacent views
    // around the tracked person until a raw gesture appears.
    static const std::vector<HeadPose> offsets = {
      {0.00, 0.00}, {0.00, -0.28},
      {0.32, -0.14}, {-0.32, -0.14}, {0.00, 0.20}
    };
    const auto &offset = offsets[
      close_gesture_search_index_ % static_cast<int>(offsets.size())];
    const double pan = std::clamp(
      close_gesture_anchor_pan_ + offset.pan, interactive_cleanup::HSR_HEAD_PAN_MIN, interactive_cleanup::HSR_HEAD_PAN_MAX);
    const double tilt = std::clamp(
      close_gesture_anchor_tilt_ + offset.tilt, interactive_cleanup::HSR_HEAD_TILT_MIN, interactive_cleanup::HSR_HEAD_TILT_MAX);
    moveHead(pan, tilt, CLOSE_GESTURE_SEARCH_MOTION_SEC);
    invalidatePickupPointingForHeadMotion(
      now_time, CLOSE_GESTURE_SEARCH_MOTION_SEC);
    if (step_ == WaitForCleanCommand) {
      destination_image_pointing_history_.clear();
      destination_camera_ready_time_ = now_time +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
            CLOSE_GESTURE_SEARCH_MOTION_SEC +
            DESTINATION_HEAD_SETTLE_SEC));
    }
    ++close_gesture_search_index_;
    last_close_gesture_search_time_ = now_time;
    RCLCPP_INFO(
      this->get_logger(),
      "[CloseGestureSearch] Person visible but hand gesture missing; "
      "expanding view pan=%.2f tilt=%.2f", pan, tilt);
  }

  void searchAvatarWithHead()
  {
    const auto now_time = std::chrono::steady_clock::now();
    if (now_time < head_motion_ready_time_ ||
        (last_person_seen_time_.time_since_epoch().count() != 0 &&
         std::chrono::duration<double>(now_time-last_person_seen_time_).count() <=
           AVATAR_OBSERVATION_MEMORY_SEC)) return;
    // A brief pose dropout during a local sweep must not immediately replace
    // that trajectory with a centre-facing global search.
    if (now_time < pointing_camera_ready_time_ ||
        (last_close_gesture_search_time_.time_since_epoch().count() != 0 &&
         std::chrono::duration<double>(
           now_time-last_close_gesture_search_time_).count() <
           CLOSE_GESTURE_SEARCH_INTERVAL_SEC)) return;
    if (last_avatar_search_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(
          now_time - last_avatar_search_time_).count() <
          AVATAR_SEARCH_INTERVAL_SEC) {
      return;
    }
    // Start from centre, then cover both sides without rotating the base.
    // The moment a person observation arrives WaitForPick switches back to
    // closed-loop tracking and this search stops.
    static const std::vector<HeadPose> search_pattern = {
      {0.00, 0.00}, {0.45, 0.00}, {-0.45, 0.00},
      {0.85, -0.05}, {-0.85, -0.05}, {0.00, 0.00}
    };
    const auto &pose = search_pattern[
      avatar_search_pose_index_ % static_cast<int>(search_pattern.size())];
    moveHead(pose.pan, pose.tilt, AVATAR_SEARCH_MOTION_SEC);
    invalidatePickupPointingForHeadMotion(
      now_time, AVATAR_SEARCH_MOTION_SEC);
    ++avatar_search_pose_index_;
    last_avatar_search_time_ = now_time;
    RCLCPP_INFO(
      this->get_logger(),
      "[AvatarSearch] No recent person observation; scanning head pan=%.2f tilt=%.2f",
      pose.pan, pose.tilt);
  }

  void reacquireAvatarForDestination()
  {
    if (hasRecentVisibleAvatar()) {
      stopBase();
      // Captured odom planes survive head motion, so following the moving
      // person no longer competes with retaining destination evidence.
      const bool tracked = trackAvatarWithHead();
      if (!tracked && !hasRecentRawPointing()) {
        searchGestureAroundVisibleAvatar();
      }
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

  void addDestinationImageEvidence(
    std::vector<interactive_cleanup::DestinationRegion> &regions)
  {
    geometry_msgs::msg::TransformStamped camera_from_odom;
    bool transform_found = false;
    for (const char *camera_frame : {
        "head_rgbd_sensor_rgb_frame",
        "head_rgbd_sensor_link",
        "rgbd_sensor_rgb_frame"}) {
      try {
        camera_from_odom = tf_buffer_->lookupTransform(
          camera_frame, "odom", tf2::TimePointZero);
        transform_found = true;
        break;
      } catch (const tf2::TransformException &) {
        continue;
      }
    }
    if (!transform_found) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PointDest/Image] Camera transform unavailable; using world rays only");
      return;
    }

    constexpr double fx = 554.0;
    constexpr double fy = 554.0;
    constexpr double cx = 320.0;
    constexpr double cy = 240.0;

    // Use one identical set of recent second-gesture samples for every
    // destination.  This avoids giving a candidate an accidental advantage
    // merely because fewer of the same gesture frames were evaluated for it.
    std::vector<const PointingDirection *> image_pointings;
    for (int i = static_cast<int>(destination_image_pointing_history_.size()) - 1;
         i >= 0 && image_pointings.size() < 10; --i) {
      const auto &pointing = destination_image_pointing_history_[i];
      const double dx = pointing.point_pixel_x - pointing.wrist_pixel_x;
      const double dy = pointing.point_pixel_y - pointing.wrist_pixel_y;
      if (pointing.is_valid && pointing.confidence >= 0.25 &&
          std::hypot(dx, dy) >= 6.0) {
        image_pointings.push_back(&pointing);
      }
    }
    if (image_pointings.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PointDest/Image] No usable second-gesture image ray; "
        "using world rays only");
      return;
    }

    for (auto &region : regions) {
      geometry_msgs::msg::PointStamped odom_point;
      odom_point.header.frame_id = "odom";
      odom_point.point = region.center;
      geometry_msgs::msg::PointStamped camera_point;
      tf2::doTransform(odom_point, camera_point, camera_from_odom);
      region.image_projection_valid = true;
      region.image_supporting_samples =
        static_cast<int>(image_pointings.size());
      if (!std::isfinite(camera_point.point.z) || camera_point.point.z <= 0.05) {
        // A destination behind the held camera cannot be the place indicated
        // by the visible forward arm ray.  Mark it explicitly instead of
        // letting it fall back to a differently scaled world-only cost.
        region.image_ray_distance_px = 9999.0;
        RCLCPP_INFO(
          this->get_logger(),
          "[PointDest/Image] '%s' is behind the held camera",
          region.name.c_str());
        continue;
      }

      const double u = fx * camera_point.point.x / camera_point.point.z + cx;
      const double v = fy * camera_point.point.y / camera_point.point.z + cy;
      std::vector<double> image_distances;
      for (const auto *pointing_ptr : image_pointings) {
        const auto &pointing = *pointing_ptr;
        const double dx = pointing.point_pixel_x - pointing.wrist_pixel_x;
        const double dy = pointing.point_pixel_y - pointing.wrist_pixel_y;
        image_distances.push_back(pointRayDist2D(
          u, v, pointing.wrist_pixel_x, pointing.wrist_pixel_y, dx, dy));
      }

      const std::size_t middle = image_distances.size() / 2;
      std::nth_element(
        image_distances.begin(), image_distances.begin() + middle,
        image_distances.end());
      double median_distance = image_distances[middle];
      if (image_distances.size() % 2 == 0) {
        median_distance = 0.5 * (
          median_distance + *std::max_element(
            image_distances.begin(), image_distances.begin() + middle));
      }
      region.image_ray_distance_px = median_distance;
      RCLCPP_INFO(
        this->get_logger(),
        "[PointDest/Image] '%s' projected=(%.0f,%.0f) ray=%.1fpx support=%d",
        region.name.c_str(), u, v, median_distance,
        region.image_supporting_samples);
    }
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

  static bool isReachableGraspHeight(double z)
  {
    return std::isfinite(z) &&
           z >= TARGET_MIN_GRASP_HEIGHT && z <= TARGET_MAX_GRASP_HEIGHT;
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
    // Object observations are cleared as soon as the target is locked, so
    // these newest rays belong to the second (destination) gesture. Keep a
    // slightly wider window than before; the resolver itself extracts the
    // dominant direction cluster and rejects transition/outlier frames.
    std::vector<interactive_cleanup::PointingObservation> pointings;
    for (int i = static_cast<int>(destination_pointing_history_.size()) - 1;
         i >= 0 && pointings.size() < 10; --i) {
      const auto &pt = destination_pointing_history_[i];
      if (pt.is_valid && pt.confidence >= 0.25) {
        interactive_cleanup::PointingObservation sample;
        sample.is_valid = true;
        sample.confidence = pt.confidence;
        sample.origin = pt.origin;
        sample.measured_depth = pt.depth_direction_valid;
        sample.plane_valid = pt.plane_valid;
        sample.plane_origin = pt.plane_origin;
        sample.plane_normal = pt.plane_normal;
        sample.plane_forward = pt.plane_forward;
        sample.camera_forward = pt.camera_forward;
        // Prefer synchronized multi-point RGB-D geometry. Pose-world remains
        // a fallback, but never bypasses the resolver's 3-D plausibility gate.
        sample.direction = pt.depth_direction_valid
          ? pt.direction
          : (pt.pose_direction_valid ? pt.pose_direction : pt.direction);
        pointings.push_back(sample);
      }
    }

    std::vector<interactive_cleanup::DestinationRegion> regions;
    regions.reserve(KNOWN_DESTINATIONS_UNITY.size());
    for (const auto &[name, unity_coordinate] : KNOWN_DESTINATIONS_UNITY) {
      const double root_x = unity_coordinate.second - HSR_INITIAL_UNITY_Z;
      const double root_y = -(unity_coordinate.first - HSR_INITIAL_UNITY_X);
      const auto profile = getPlacementProfile(name);
      auto region = interactive_cleanup::inferDestinationRegion(name, root_x, root_y);
      // Match the actual JudgeTriggerOn/JudgeTriggerIn centre used later for
      // release, not a zero-height floor projection of the furniture root.
      region.center.x += profile.center_offset_x;
      region.center.y += profile.center_offset_y;
      region.center.z = profile.release_height;
      regions.push_back(region);
    }

    // Tracking continues throughout the second gesture. The callback keeps
    // only samples gathered after the latest head correction has settled, so
    // project destinations into that matching final camera view.
    const bool captured_plane_available = std::any_of(
      pointings.begin(), pointings.end(),
      [](const auto &pointing) { return pointing.plane_valid; });
    if (!captured_plane_available) addDestinationImageEvidence(regions);

    const auto resolution = interactive_cleanup::resolvePlaceDestination(
      regions, pointings, target_class_);
    if (!resolution.valid) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PointDest] Rejected destination gesture: %s (usable rays=%d)",
        resolution.rejection_reason.c_str(), resolution.usable_samples);
      return false;
    }
    if (resolution.horizontal_bearing_used) {
      RCLCPP_INFO(this->get_logger(),
        "[PointDest] Measured horizontal bearing consensus: '%s' support=%d/%d",
        resolution.name.c_str(), resolution.supporting_samples, resolution.usable_samples);
    } else if (captured_plane_available) {
      RCLCPP_INFO(this->get_logger(),
        "[PointDest] Capture-time image-plane/depth consensus: '%s' support=%d/%d",
        resolution.name.c_str(), resolution.supporting_samples, resolution.usable_samples);
    }

    const auto &unity_coordinate = KNOWN_DESTINATIONS_UNITY.at(resolution.name);
    dest_position_.x = unity_coordinate.second - HSR_INITIAL_UNITY_Z;
    dest_position_.y = -(unity_coordinate.first - HSR_INITIAL_UNITY_X);
    dest_position_.z = 0.0;
    dest_valid_ = true;
    dest_class_ = resolution.name;
    if (!captured_plane_available) RCLCPP_INFO(
      this->get_logger(),
      "[PointDest] Selected '%s' root odom=(%.2f, %.2f), "
      "3-D trigger=(%.2f, %.2f, %.2f), Unity=(%.2f, %.2f): "
      "image_ray=%.1fpx(%s) planar_ray=%.2fm 3d_ray=%.2fm surface_hit=%.2fm "
      "along=%.2fm support=%d/%d",
      dest_class_.c_str(), dest_position_.x, dest_position_.y,
      resolution.position.x, resolution.position.y, resolution.position.z,
      unity_coordinate.first, unity_coordinate.second,
      resolution.image_ray_distance_px,
      resolution.image_evidence_used ? "used" : "unanchored",
      resolution.ray_distance, resolution.three_d_ray_distance,
      resolution.surface_intersection_distance, resolution.along_distance,
      resolution.supporting_samples, resolution.usable_samples);
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
    const auto original_ray = locked_pointing_;
    const bool original_lock = has_locked_pointing_;
    const bool original_historical = locked_historical_arm_;
    const auto original_position = target_position_;
    const auto original_class = target_class_;
    const bool original_valid = target_valid_;
    std::vector<PointingDirection> hypotheses;
    const auto reference_ns = destination_window_.begin_ns > 0
      ? destination_window_.begin_ns : ingressNowNs();
    for (const auto &ray : pickup_ray_hypotheses_) {
      const auto ns = ray.received_steady_ns;
      if (ns > 0 && ns <= reference_ns && reference_ns-ns <= static_cast<int64_t>(POINTING_MEMORY_SEC*1e9))
        hypotheses.push_back(ray);
    }
    // The live target gesture may begin after Pick_it_up!; it remains target
    // evidence until Clean_up! closes the capture window. A ray arriving after
    // that boundary belongs to the destination gesture and must not vote here.
    const bool original_ray_in_target_window = destination_window_.end_ns <= 0 ||
      (original_ray.received_steady_ns > 0 &&
       original_ray.received_steady_ns <= destination_window_.end_ns);
    const bool use_original = original_lock && original_ray_in_target_window;
    if (use_original) hypotheses.insert(hypotheses.begin(), original_ray);
    bool found = false, conflict = false;
    PointingDirection winner;
    geometry_msgs::msg::Point position;
    std::string name;
    bool historical = false;
    for (size_t i=0; i<hypotheses.size(); ++i) {
      locked_pointing_ = hypotheses[i];
      has_locked_pointing_ = true;
      locked_historical_arm_ = use_original && i == 0 && original_historical;
      if (!selectStableTargetForRay(false)) continue;
      // Same instance definition as the object accumulator. Contradictory
      // supported hypotheses remain unresolved; iteration order cannot pick.
      if (found && (target_class_ != name ||
          std::lround(target_position_.x*2) != std::lround(position.x*2) ||
          std::lround(target_position_.y*2) != std::lround(position.y*2))) {
        conflict = true;
        break;
      }
      if (!found) {
        found = true;
        winner = locked_pointing_;
        position = target_position_;
        name = target_class_;
        historical = locked_historical_arm_;
      }
    }
    locked_pointing_ = original_ray;
    has_locked_pointing_ = original_lock;
    locked_historical_arm_ = original_historical;
    target_position_ = original_position;
    target_class_ = original_class;
    target_valid_ = original_valid;
    if (!found || conflict) return false;
    locked_pointing_ = winner;
    has_locked_pointing_ = true;
    locked_historical_arm_ = historical;
    // Commit once; probes above have no externally visible decision/log.
    return selectStableTargetForRay(true);
  }

  bool selectStableTargetForRay(bool report = true)
  {
    static auto last_diagnostic = std::chrono::steady_clock::time_point{};
    const auto diagnostic_now = std::chrono::steady_clock::now();
    const bool diagnose = report && diagnostic_now-last_diagnostic >= std::chrono::seconds(2);
    if (diagnose) last_diagnostic = diagnostic_now;
    struct Candidate {
      std::string name;
      double x, y, z;
      int frames;
      double confidence_sum;
      double ray_distance_sum;
      double best_ray_distance;
    };
    // Keep a compact per-class account of every candidate and each gate it
    // fails. Detection counts alone are not evidence that an object matched
    // the gesture; these aggregates make that distinction visible in logs.
    struct GeometryAudit {
      int seen = 0;
      int with_3d = 0;
      int low_confidence = 0;
      int non_portable = 0;
      int unreachable_height = 0;
      int plane_angle_rejected = 0;
      int halfspace_rejected = 0;
      int ray_rejected = 0;
      int accepted = 0;
      double z_sum = 0.0;
      double depth_mm_sum = 0.0;
      double ray_sum = 0.0;
      double ray_best = std::numeric_limits<double>::infinity();
      double along_sum = 0.0;
      double plane_sum = 0.0;
    };
    std::map<std::string, Candidate> candidates;
    std::map<std::string, GeometryAudit> geometry_audit;

    for (const auto &frame : obs_objects_) {
      if (!has_locked_pointing_) continue;
      // The object detector and pose detector are not synchronized streams.
      // Use the gesture ray that opened this observation window for every
      // object frame.  Requiring an arbitrary sub-second timestamp match here
      // discarded otherwise stable detections in the field whenever the two
      // inference loops ran at different rates.
      std::set<std::string> updated_in_frame;
      for (const auto &obj : frame.objects) {
        if (obj.class_name == "person") continue;
        auto &audit = geometry_audit[obj.class_name];
        audit.seen++;
        const bool portable = isLikelyPortableClass(obj.class_name);
        const bool reachable = obj.has_3d_position &&
          isReachableGraspHeight(obj.position_3d.z);
        if (!obj.has_3d_position) {
          continue;
        }
        audit.with_3d++;
        audit.z_sum += obj.position_3d.z;
        audit.depth_mm_sum += obj.depth_mm;
        const double world_ray_distance = pointRayDist3D(
          obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
          locked_pointing_.origin.x, locked_pointing_.origin.y,
          locked_pointing_.origin.z,
          locked_pointing_.direction.x,
          locked_pointing_.direction.y,
          locked_pointing_.direction.z);
        const double plane_angle = locked_pointing_.plane_valid
          ? interactive_cleanup::pointingPlaneAngle(
              obj.position_3d, locked_pointing_.plane_origin,
              locked_pointing_.plane_normal) : 0.0;
        const double ray_along =
          (obj.position_3d.x - locked_pointing_.origin.x) * locked_pointing_.direction.x +
          (obj.position_3d.y - locked_pointing_.origin.y) * locked_pointing_.direction.y +
          (obj.position_3d.z - locked_pointing_.origin.z) * locked_pointing_.direction.z;
        audit.ray_sum += world_ray_distance;
        audit.ray_best = std::min(audit.ray_best, world_ray_distance);
        audit.along_sum += ray_along;
        audit.plane_sum += plane_angle;
        if (obj.confidence < STABLE_TARGET_MIN_CONFIDENCE) audit.low_confidence++;
        if (!portable) audit.non_portable++;
        if (!reachable) audit.unreachable_height++;
        if (obj.confidence < STABLE_TARGET_MIN_CONFIDENCE || !portable || !reachable) {
          if (diagnose && &frame == &obs_objects_.back()) RCLCPP_INFO(this->get_logger(),
            "[TargetAudit] rejected basic filters class=%s xyz=(%.3f,%.3f,%.3f) "
            "depth_mm=%.1f confidence=%.3f portable=%d reachable=%d "
            "height_range=[%.2f,%.2f]",
            obj.class_name.c_str(), obj.position_3d.x, obj.position_3d.y,
            obj.position_3d.z, obj.depth_mm, obj.confidence, portable, reachable,
            TARGET_MIN_GRASP_HEIGHT, TARGET_MAX_GRASP_HEIGHT);
          continue;
        }

        if (diagnose && &frame == &obs_objects_.back()) {
          RCLCPP_INFO(this->get_logger(),
            "[TargetAudit] class=%s xyz=(%.3f,%.3f,%.3f) depth_mm=%.1f "
            "confidence=%.3f ray=%.3f plane=%.4f depth_ray_valid=%d plane_valid=%d",
            obj.class_name.c_str(), obj.position_3d.x, obj.position_3d.y,
            obj.position_3d.z, obj.depth_mm, obj.confidence, world_ray_distance, plane_angle,
            locked_pointing_.depth_direction_valid, locked_pointing_.plane_valid);
          RCLCPP_INFO(this->get_logger(),
            "[TargetAudit] class=%s plane_pass=%d forward_pass=%d camera_pass=%d",
            obj.class_name.c_str(), !locked_pointing_.plane_valid || plane_angle <= 0.12,
            !locked_pointing_.plane_valid || interactive_cleanup::pointingHalfspaceContains(
              obj.position_3d, locked_pointing_.plane_origin, locked_pointing_.plane_forward, 0.03),
            !locked_pointing_.plane_valid || interactive_cleanup::pointingHalfspaceContains(
              obj.position_3d, locked_pointing_.plane_origin, locked_pointing_.camera_forward));
        }
        // An image line constrains a plane, never an invented camera ray.
        // With measured arm depth both constraints must agree. Without arm
        // depth use the plane plus depth-localized objects, rejecting ties.
        if (locked_pointing_.plane_valid && plane_angle > 0.12) {
          audit.plane_angle_rejected++;
          continue;
        }
        if (locked_pointing_.plane_valid &&
            (!interactive_cleanup::pointingHalfspaceContains(
               obj.position_3d, locked_pointing_.plane_origin,
               locked_pointing_.plane_forward, 0.03) ||
             !interactive_cleanup::pointingHalfspaceContains(
               obj.position_3d, locked_pointing_.plane_origin,
               locked_pointing_.camera_forward))) {
          audit.halfspace_rejected++;
          continue;
        }
        const double ray_distance = locked_pointing_.depth_direction_valid
          ? world_ray_distance
          : (locked_pointing_.plane_valid ? plane_angle * 3.0 : world_ray_distance);
        if (ray_distance > 1.5) {
          audit.ray_rejected++;
          continue;
        }

        // Keep separate instances of the same class separate.  The failed
        // run contained two bear_doll detections per frame; grouping only by
        // class produced 60 "frames" from a 30-frame window and averaged two
        // physical positions into a nonexistent target between them.
        const std::string spatial_key =
          obj.class_name + "_" +
          std::to_string(std::lround(obj.position_3d.x * 2.0)) + "_" +
          std::to_string(std::lround(obj.position_3d.y * 2.0));
        if (updated_in_frame.count(spatial_key) != 0) continue;
        updated_in_frame.insert(spatial_key);
        audit.accepted++;
        auto result = candidates.emplace(
          spatial_key,
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
    double runner_up_score = -1.0;
    double best_geometry = std::numeric_limits<double>::infinity();
    double runner_up_geometry = std::numeric_limits<double>::infinity();
    double best_score = -1.0;
    if (diagnose) {
      for (const auto &[class_name, audit] : geometry_audit) {
        const double n3d = static_cast<double>(std::max(1, audit.with_3d));
        RCLCPP_INFO(this->get_logger(),
          "[TargetGeometryAudit] class=%s seen=%d with_3d=%d xyz_z_mean=%.3f depth_mm_mean=%.1f "
          "ray_mean=%.3f ray_best=%.3f along_mean=%.3f plane_angle_mean=%.4f "
          "reject_conf=%d reject_class=%d reject_height=%d reject_plane=%d "
          "reject_halfspace=%d reject_ray=%d accepted=%d",
          class_name.c_str(), audit.seen, audit.with_3d,
          audit.z_sum / n3d, audit.with_3d ? audit.depth_mm_sum / n3d : -1.0,
          audit.with_3d ? audit.ray_sum / n3d : -1.0,
          audit.with_3d ? audit.ray_best : -1.0,
          audit.with_3d ? audit.along_sum / n3d : -1.0,
          audit.with_3d ? audit.plane_sum / n3d : -1.0,
          audit.low_confidence, audit.non_portable, audit.unreachable_height,
          audit.plane_angle_rejected, audit.halfspace_rejected,
          audit.ray_rejected, audit.accepted);
      }
    }
    for (const auto &[key, candidate] : candidates) {
      const double candidate_average_ray =
        candidate.ray_distance_sum / candidate.frames;
      if (diagnose) RCLCPP_INFO(this->get_logger(),
        "[TargetVoteAudit] key=%s class=%s frames=%d required_frames=%d "
        "best_ray=%.3f max_best=%.3f avg_ray=%.3f max_avg=%.3f",
        key.c_str(), candidate.name.c_str(), candidate.frames,
        STABLE_TARGET_MIN_FRAMES, candidate.best_ray_distance,
        STABLE_TARGET_MAX_BEST_RAY_3D_M, candidate_average_ray,
        STABLE_TARGET_MAX_AVG_RAY_3D_M);
      if (candidate.frames < STABLE_TARGET_MIN_FRAMES) continue;
      const double average_confidence =
        candidate.confidence_sum / candidate.frames;
      const double average_ray_distance = candidate_average_ray;
      if (candidate.best_ray_distance > STABLE_TARGET_MAX_BEST_RAY_3D_M ||
          average_ray_distance > STABLE_TARGET_MAX_AVG_RAY_3D_M) {
        continue;
      }
      if (average_ray_distance < best_geometry) {
        runner_up_geometry = best_geometry;
        best_geometry = average_ray_distance;
      } else {
        runner_up_geometry = std::min(runner_up_geometry, average_ray_distance);
      }
      const double score =
        std::pow(static_cast<double>(candidate.frames), 1.25) *
        average_confidence * std::exp(-average_ray_distance / 1.2);
      if (score > best_score) {
        runner_up_score = best_score;
        best_score = score;
        best = &candidate;
      } else {
        runner_up_score = std::max(runner_up_score, score);
      }
    }
    // Keep the selector used by the successful task-1 version as the only
    // decision path. Broad recovery rays can turn a merely persistent object
    // from the later destination view into a false pickup target.
    if (diagnose) RCLCPP_INFO(this->get_logger(),
      "[TargetDecisionAudit] best=%s score=%.3f runner_score=%.3f geometry_gap=%.3f",
      best ? best->name.c_str() : "none", best_score, runner_up_score,
      runner_up_geometry-best_geometry);
    if (best == nullptr) return false;
    // An image-line plane can also contain objects behind the pointing hand.
    // Check this even for a unique winner: ambiguity handling below is not
    // reached when the actual target was rejected by the image-plane gate.
    // Pose is only a coarse contradiction veto, never metric depth evidence.
    const auto &pose = locked_pointing_.pose_direction;
    const double pose_horizontal = std::hypot(pose.x, pose.y);
    const double pose_norm = std::hypot(pose_horizontal, pose.z);
    const double hand_dx = best->x - locked_pointing_.origin.x;
    const double hand_dy = best->y - locked_pointing_.origin.y;
    const double hand_range = std::hypot(hand_dx, hand_dy);
    if (!locked_pointing_.depth_direction_valid && locked_pointing_.plane_valid &&
        locked_pointing_.pose_direction_valid && std::isfinite(pose_norm) &&
        pose_norm > 1e-6 && pose_horizontal / pose_norm >= 0.25 && hand_range >= 0.15) {
      const double bearing = (hand_dx*pose.x + hand_dy*pose.y) /
        (hand_range*pose_horizontal);
      if (bearing <= -0.5) {
        if (diagnose) RCLCPP_WARN(this->get_logger(),
          "[TargetDecisionAudit] Rejecting image-plane winner '%s': "
          "opposite arm bearing %.3f; gesture evidence conflicts",
          best->name.c_str(), bearing);
        return false;
      }
    }
    const bool ambiguous =
      (!locked_pointing_.depth_direction_valid && locked_pointing_.plane_valid &&
       runner_up_geometry - best_geometry < 0.09) ||
      (runner_up_score > 0.0 && runner_up_score > best_score * 0.85);
    if (ambiguous) {
      // A camera-origin plane can contain objects on both sides of the hand.
      // Use pose ONLY as coarse front/back corroboration, never as measured
      // depth or as a replacement for image ranking. Require agreement with
      // both image rankings and reject all same-side/uncertain competitors.
      const auto &p = locked_pointing_;
      const double horizontal = std::hypot(p.direction.x, p.direction.y);
      const double norm = std::hypot(horizontal, p.direction.z);
      auto bearing = [&](const Candidate &c) {
        const double x = c.x-p.origin.x, y = c.y-p.origin.y;
        const double range = std::hypot(x,y);
        if (range < 0.15 || horizontal < 1e-6) return 0.0;
        return (x*p.direction.x+y*p.direction.y)/(range*horizontal);
      };
      bool corroborated = !p.depth_direction_valid && p.plane_valid &&
        p.pose_direction_valid && p.confidence >= 0.5 &&
        std::isfinite(norm) && norm > 1e-6 && horizontal/norm >= 0.25 &&
        best->ray_distance_sum/best->frames <= best_geometry+1e-6 &&
        bearing(*best) >= 0.5;
      for (const auto &[key, c] : candidates) {
        if (&c == best || c.frames < STABLE_TARGET_MIN_FRAMES ||
            c.best_ray_distance > STABLE_TARGET_MAX_BEST_RAY_3D_M ||
            c.ray_distance_sum/c.frames > STABLE_TARGET_MAX_AVG_RAY_3D_M) continue;
        if (!(bearing(c) <= -0.5)) corroborated = false;
      }
      if (!corroborated) return false;
      if (report) RCLCPP_INFO(this->get_logger(),
        "[ObserveTarget] Image winner corroborated by coarse arm bearing; "
        "all competing candidates are behind the hand (pose-only, not depth)");
    }


    target_position_.x = best->x;
    target_position_.y = best->y;
    target_position_.z = best->z;
    target_valid_ = true;
    target_class_ = best->name;
    if (report) RCLCPP_INFO(this->get_logger(),
                "[ObserveTarget] Stable pointed target: class='%s' pos=(%.2f, %.2f, %.2f) frames=%d geometry_cost=%.3f evidence=%s score=%.2f",
                target_class_.c_str(), target_position_.x, target_position_.y,
                target_position_.z, best->frames,
                best->ray_distance_sum / best->frames,
                locked_historical_arm_ ? "retained_full_arm" :
                (locked_pointing_.depth_direction_valid ? "rgbd+image_plane" : "image_plane"),
                best_score);
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
    // A distractor must not prevent the one-shot reduced-view recovery.
    // Keep the original locked gesture; do not reacquire a later gesture.
    if (!has_locked_pointing_) return false;
    for (const auto &frame : obs_objects_) {
      for (const auto &obj : frame.objects) {
        if (!obj.has_3d_position || obj.confidence < STABLE_TARGET_MIN_CONFIDENCE ||
            !isLikelyPortableClass(obj.class_name) ||
            !isReachableGraspHeight(obj.position_3d.z)) continue;
        const auto &p = locked_pointing_;
        if (p.plane_valid) {
          if (interactive_cleanup::pointingPlaneAngle(
                obj.position_3d, p.plane_origin, p.plane_normal) > 0.12 ||
              !interactive_cleanup::pointingHalfspaceContains(
                obj.position_3d, p.plane_origin, p.plane_forward, 0.03) ||
              !interactive_cleanup::pointingHalfspaceContains(
                obj.position_3d, p.plane_origin, p.camera_forward)) continue;
          if (!p.depth_direction_valid) return true;
        }
        if (pointRayDist3D(obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
              p.origin.x, p.origin.y, p.origin.z,
              p.direction.x, p.direction.y, p.direction.z) <= 0.55) return true;
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
      // The bin outer footprint is about 0.50 x 0.60 m. An 8 cm
      // approach-side offset stays within that footprint and covers the
      // 6.2 cm residual seen when the base-clearance guard stopped the last
      // release attempt.
      return {0.55, 0.08, 0.0, 0.0, true};
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

    // Use a point slightly toward the robot. It remains inside the placement
    // region while shortening the reach from the safe approach pose.
    placement_position_.x =
      center_x + placement_profile_.approach_inset * ux;
    placement_position_.y =
      center_y + placement_profile_.approach_inset * uy;
    placement_position_.z = placement_profile_.release_height;
    placement_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "[Placement] '%s' target=(%.2f, %.2f, %.2f) mode=%s approach_inset=%.2fm",
      dest_class_.c_str(), placement_position_.x, placement_position_.y,
      placement_position_.z,
      placement_profile_.drop_in ? "drop-in" : "surface",
      placement_profile_.approach_inset);
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
    if (ground_norm < 1e-3 && !last_valid_pointing_.plane_valid) {
      return false;
    }

    pt = last_valid_pointing_;
    return true;
  }

  bool lockRecentPointing()
  {
    if (has_locked_pointing_) return true;
    locked_historical_arm_ = false;
    if (lockCurrentPointing()) return true;
    // Command-driven fallback only: keep stable pre-Pick preview unchanged.
    if (step_ != ObserveTarget || destination_window_.begin_ns == 0 ||
        !has_retained_arm_pointing_) return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now()-retained_arm_time_).count();
    if (age > POINTING_MEMORY_SEC) return false;
    locked_pointing_ = retained_arm_pointing_;
    // Historical endpoint/pose geometry is an estimate, not a depth fit.
    // Rank by spatial ray distance as d902b25 did, without the image-plane
    // veto that becomes unstable for foreshortened full arms.
    locked_pointing_.depth_direction_valid = false;
    locked_pointing_.plane_valid = false;
    has_locked_pointing_ = true;
    locked_historical_arm_ = true;
    // Retention is not confirmation: a walking/turning arm can be the last
    // full-arm observation. Test it against the accumulated object frames
    // before stopping the live gesture/head search.
    if (!selectStableTargetForRay(false)) {
      has_locked_pointing_ = false;
      locked_historical_arm_ = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), perception_stamp_clock_, 2000,
        "[PickupMemory] Retained ray age=%.2fs has no stable object match; "
        "continuing live gesture search", age);
      return false;
    }
    close_gesture_search_active_ = false;
    close_gesture_search_index_ = 0;
    RCLCPP_INFO(this->get_logger(),
      "[PickupMemory] Using pre-Pick full-arm estimate age=%.2fs stamp=%.9f; "
      "confirmed by multi-frame object agreement", age,
      rclcpp::Time(locked_pointing_.header.stamp).seconds());
    return true;
  }

  bool buildCurrentPointing(PointingDirection &pt)
  {
    if (!getRecentValidPointing(pt)) {
      if (hasRecentRawPointing()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "[ObserveTarget] Raw gesture exists but no settled valid world ray "
          "is retained yet");
      }
      return false;
    }
    const double image_length = std::hypot(
      pt.point_pixel_x - pt.wrist_pixel_x,
      pt.point_pixel_y - pt.wrist_pixel_y);
    if ((!std::isfinite(image_length) || image_length < MIN_PICKUP_IMAGE_SEGMENT) &&
        (!pt.depth_direction_valid || pt.depth_direction_samples < 4)) {
      // Do not permanently lock a foreshortened pose-only estimate. Keep
      // observing the tracked person until a real depth fit or a visible
      // finger segment arrives.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "[ObserveTarget] Rejecting foreshortened fallback ray: "
        "image_segment=%.1fpx depth_valid=%s depth_samples=%u",
        image_length, pt.depth_direction_valid ? "true" : "false",
        static_cast<unsigned>(pt.depth_direction_samples));
      has_last_valid_pointing_ = false;
      return false;
    }
    if (!pt.depth_direction_valid) {
      // A pose-only direction must not become a permanent target ray from
      // one short-lived hand detection. Use the same temporal confirmation
      // as pre-command observation, restricted to usable image segments.
      std::vector<PointingDirection> samples;
      for (const auto &sample : pickup_pointing_history_) {
        const double length = std::hypot(
          sample.point_pixel_x - sample.wrist_pixel_x,
          sample.point_pixel_y - sample.wrist_pixel_y);
        if (sample.geometry_source == pt.geometry_source &&
            !sample.depth_direction_valid && std::isfinite(length) &&
            length >= MIN_PICKUP_IMAGE_SEGMENT) samples.push_back(sample);
      }
      if (samples.empty() || samples.back().header.stamp != pt.header.stamp ||
          !interactive_cleanup::stablePickupPreview(samples)) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), perception_stamp_clock_, 2000,
          "[GestureAudit] image ray not locked: waiting for 3 consistent frames over 0.6s");
        return false;
      }
    }
    if (pt.depth_direction_valid) {
      int support = 0;
      for (const auto &sample : pickup_pointing_history_) {
        if (!sample.depth_direction_valid || sample.geometry_source != pt.geometry_source) continue;
        const double age = std::abs((rclcpp::Time(pt.header.stamp) -
          rclcpp::Time(sample.header.stamp)).seconds());
        const double dot = pt.direction.x*sample.direction.x +
          pt.direction.y*sample.direction.y + pt.direction.z*sample.direction.z;
        // Pose/hand inference can be spaced 2–3 s apart in the simulator.
        // Keep a short multi-frame confirmation window long enough to contain
        // two distinct depth fits without relaxing the angular agreement.
        if (age > 4.0 || dot < std::cos(0.35)) continue;
        ++support;
      }
      // A single frame with low fit error is not temporal confirmation.
      if (support < 2) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), perception_stamp_clock_, 2000,
          "[GestureAudit] depth ray not locked: support=%d required=2", support);
        return false;
      }
      // History corroborates this observation; it must not manufacture a
      // mean 3-D ray paired with a plane/pose from only the newest frame.
      // Keep origin, depth direction and image plane from one capture.
    }
    return true;
  }

  bool lockCurrentPointing()
  {
    PointingDirection pt;
    if (!buildCurrentPointing(pt)) return false;
    locked_pointing_ = pt;
    has_locked_pointing_ = true;
    // This is a provisional observation direction, not a confirmed object.
    // Only confirmed evidence completes a search. Isolated crop detections
    // must not repeatedly restart the sweep at its first two views.
    close_gesture_search_active_ = false;
    close_gesture_search_index_ = 0;
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
    avatar_search_pose_index_ = 0;
    last_avatar_search_time_ = {};
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
    early_pick_attempted_ = false;
    early_pick_observing_ = false;
    preconfirmed_target_ = false;
    preconfirmed_target_time_ = {};
    clean_drain_deadline_ = {};
    destination_window_ = {};
    received_yes_ = false;
    received_no_ = false;
    state_timer_initialized_ = false;
    sub_step_ = 0;
    scan_index_ = 0;
    point_align_retry_count_ = 0;
    ready_ack_sent_ = false;
    has_new_objects_ = false;
    has_new_pointing_ = false;
    has_last_valid_pointing_ = false;
    has_retained_arm_pointing_ = false;
    retained_arm_time_ = {};
    locked_historical_arm_ = false;
    pickup_pointing_history_.clear();
    pickup_ray_hypotheses_.clear();
    head_observation_not_before_ns_ = 0;
    head_motion_ready_time_ = {};
    has_locked_pointing_ = false;
    has_person_observation_ = false;
    last_person_seen_time_ = {};
    last_person_observation_time_ = {};
    last_raw_pointing_time_ = {};
    last_close_gesture_search_time_ = {};
    last_person_pixel_x_ = 320.0;
    last_person_pixel_y_ = 240.0;
    last_avatar_track_time_ = {};
    last_avatar_search_time_ = {};
    avatar_search_pose_index_ = 0;
    close_gesture_search_index_ = 0;
    close_gesture_search_active_ = false;
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
    destination_pointing_history_.clear();
    destination_image_pointing_history_.clear();
    destination_capture_closed_ = false;
    destination_gesture_started_ = false;
    pointing_camera_ready_time_ = {};
    destination_camera_ready_time_ = {};
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
      if (is_started_ && vision_ready_) {
        armObserve();
        headCenter();
        pointing_camera_ready_time_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
              1.0 + DESTINATION_HEAD_SETTLE_SEC));
        sendReadyAck("transition to task start");
        enableVision(true);
        changeStep(WaitForPickCommand);
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForPickCommand: {
      // Start the existing target observation algorithm during a stable first
      // gesture. This is perception only; no pickup or protocol claim yet.
      if (!received_pick_command_ && !early_pick_attempted_ &&
          hasRecentVisibleAvatar() && hasRecentRawPointing(1.5) &&
          interactive_cleanup::stablePickupPreview(pickup_pointing_history_) &&
          lockRecentPointing()) {
        early_pick_attempted_ = true;
        early_pick_observing_ = true;
        obs_objects_.clear();
        obs_pointings_.clear();
        target_view_recovery_count_ = 0;
        target_aim_pose_valid_ = false;
        RCLCPP_INFO(this->get_logger(),
          "[PrePick] Stable gesture for >=0.6s; observing target before command");
        changeStep(ObserveTarget);
        break;
      }
      if (hasRecentVisibleAvatar()) {
        // A visible hand does not mean the person has stopped moving.
        // Continue following; acquire the pickup ray once the view settles.
        const bool tracked = trackAvatarWithHead();
        if (!tracked && !hasRecentRawPointing() && elapsed() >= 2.0) {
          searchGestureAroundVisibleAvatar();
        }
      } else {
        searchAvatarWithHead();
      }
      if (received_pick_command_) {
        RCLCPP_INFO(this->get_logger(), "[WaitForPick] Got Pick_it_up! → observing target");
        received_pick_command_ = false;
        if (preconfirmed_target_ && target_valid_ &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
              preconfirmed_target_time_).count() <= POINTING_MEMORY_SEC) {
          preconfirmed_target_ = false;
          destination_pointing_history_.clear();
          destination_image_pointing_history_.clear();
          destination_capture_closed_ = false;
          destination_gesture_started_ = false;
          RCLCPP_INFO(this->get_logger(),
            "[PrePick] Reusing confirmed '%s'; destination capture starts immediately",
            target_class_.c_str());
          changeStep(WaitForCleanCommand);
          break;
        }
        preconfirmed_target_ = false;
        target_valid_ = false;
        destination_pointing_history_.clear();
        destination_image_pointing_history_.clear();
        destination_capture_closed_ = false;
        destination_gesture_started_ = false;
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
          if (aimHeadAlongPointing(locked_pointing_)) {
            obs_objects_.clear();
            sub_step_ = 1;
            enterTimedState();
          } else {
            beginPointingHold("ObserveTarget");
            sub_step_ = 2;
          }
        } else {
          beginPointingWait("ObserveTarget");
          sub_step_ = 4;
        }
      } else if (sub_step_ == 4) {
        if (lockRecentPointing()) {
          RCLCPP_INFO(this->get_logger(),
                      "[ObserveTarget] Pointing acquired; holding current view");
          // Give target voting a complete window from the retained first
          // gesture and move the head once along its image-space direction.
          state_enter_time_ = std::chrono::steady_clock::now();
          if (aimHeadAlongPointing(locked_pointing_)) {
            obs_objects_.clear();
            sub_step_ = 1;
          } else {
            beginPointingHold("ObserveTarget");
            sub_step_ = 2;
          }
        } else if (hasRecentVisibleAvatar() && hasRecentRawPointing() &&
                   personOutsideGestureCaptureEnvelope()) {
          trackAvatarWithHead();
        } else if (hasRecentVisibleAvatar() && !hasRecentRawPointing() &&
                   elapsed() >= 0.5) {
          if (!trackAvatarWithHead()) {
            searchGestureAroundVisibleAvatar();
          }
        } else if (!hasRecentRawPointing()) {
          if (hasRecentVisibleAvatar()) {
            trackAvatarWithHead();
          } else {
            searchAvatarWithHead();
          }
        }
      } else if (sub_step_ == 1 && elapsed() >= 1.0) {
        // Discard frames captured during this single head trajectory.
        obs_objects_.clear();
        beginPointingHold("ObserveTarget");
        enterTimedState();
        sub_step_ = 2;
      } else if (sub_step_ == 2 && elapsed() >= 3.0 &&
                 !hasObservedNonPersonObject() &&
                 target_aim_pose_valid_ && target_view_recovery_count_ == 0) {
        const double recovery_pan =
          0.5 * (target_aim_origin_pan_ + target_aim_pan_);
        const double recovery_tilt = std::min(
          0.5 * (target_aim_origin_tilt_ + target_aim_tilt_), -0.25);
        moveHead(recovery_pan, recovery_tilt, 0.8);
        target_view_recovery_count_ = 1;
        obs_objects_.clear();
        RCLCPP_WARN(
          this->get_logger(),
          "[ObserveTarget] Directed view contains no objects consistent with the locked gesture; "
          "trying one reduced same-direction view pan=%.2f tilt=%.2f",
          recovery_pan, recovery_tilt);
        sub_step_ = 6;
        enterTimedState();
      } else if (sub_step_ == 6 && elapsed() >= 1.0) {
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
        destination_pointing_history_.clear();
        destination_image_pointing_history_.clear();
        destination_capture_closed_ = false;
        destination_gesture_started_ = false;
        // Prepare to observe the destination. Before Pick_it_up!, this is
        // preparation only: destination_window_ still rejects those frames.
        // destination_camera_ready_time_ below is the guard against retaining
        // the pickup view while the head returns to the Avatar. Requiring an
        // inferred release lost valid second gestures in real runs.
        has_last_valid_pointing_ = false;
        has_locked_pointing_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "[ObserveTarget] Target confirmed without camera scan");
        // The directed target view usually no longer contains the Avatar.
        // Return the head to the person before the second gesture begins;
        // WaitForClean then continuously follows the moving Avatar and only
        // votes with samples captured after each correction has settled.
        headCenter();
        destination_camera_ready_time_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 + DESTINATION_HEAD_SETTLE_SEC));
        if (early_pick_observing_ && !received_pick_command_) {
          invalidatePickupPointingForHeadMotion(
            std::chrono::steady_clock::now(), 1.0);
          early_pick_observing_ = false;
          preconfirmed_target_ = true;
          preconfirmed_target_time_ = std::chrono::steady_clock::now();
          RCLCPP_INFO(this->get_logger(),
            "[PrePick] Cached '%s'; returning to person while awaiting Pick_it_up!",
            target_class_.c_str());
          changeStep(WaitForPickCommand);
        } else {
          RCLCPP_INFO(this->get_logger(),
            "[WaitForClean] Target confirmed; collecting post-Pick destination evidence");
          early_pick_observing_ = false;
          received_pick_command_ = false;
          changeStep(WaitForCleanCommand);
        }
        break;
      }

      // Clean_up! is the hard boundary between the target and destination
      // gestures.  Never keep observing and later reinterpret destination
      // objects as the pickup target, which is what selected hourglass in the
      // failed run.  A valid target must already have been confirmed above.
      if (received_clean_command_) {
        received_clean_command_ = false;
        stopBase();
        RCLCPP_ERROR(
          this->get_logger(),
          "[ObserveTarget] Clean_up! arrived before a target was confirmed; "
          "refusing to mix destination frames into target selection");
        giveUp();
        changeStep(WaitForResult);
        break;
      }

      const double target_observe_timeout =
        (sub_step_ == 4) ? TIMEOUT_POINTING_WAIT :
        (sub_step_ == 7) ? 5.0 : TIMEOUT_OBSERVE;
      if (elapsed() >= target_observe_timeout) {
        if (early_pick_observing_ && !received_pick_command_ &&
            !received_clean_command_) {
          early_pick_observing_ = false;
          has_locked_pointing_ = false;
          has_last_valid_pointing_ = false;
          pickup_pointing_history_.clear();
          obs_objects_.clear();
          headCenter();
          invalidatePickupPointingForHeadMotion(
            std::chrono::steady_clock::now(), 1.0);
          RCLCPP_INFO(this->get_logger(),
            "[PrePick] Preview inconclusive; returning to normal command-driven recognition");
          changeStep(WaitForPickCommand);
          break;
        }
        stopBase();
        logObservationSummary("ObserveTarget");
        logPointingRayDiagnostics("ObserveTarget");
        // Interactive Cleanup requires the object indicated by the finger.
        // Do not silently substitute the highest-scoring YOLO box when no
        // stable finger/object match was obtained.
        RCLCPP_WARN(this->get_logger(),
                    "[ObserveTarget] No stable finger-to-object match in observation window");
        // The moderator proceeds to the destination gesture after Pick.  An
        // automatic target re-point here is unsafe because that second
        // gesture can then be mistaken for a pickup object. Fail this task
        // cleanly instead of crossing the phase boundary.
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForCleanCommand: {
      // Follow the Avatar for the entire transition and destination gesture.
      // Any head correction invalidates older pixel rays; the callback starts
      // a fresh vote after the camera settles instead of freezing the view.
      if (!received_clean_command_) reacquireAvatarForDestination();

      if (received_clean_command_) {
        // Drain inference already in flight, not new post-command gestures.
        // pointingCallback gates every result by its capture timestamp.
        if (std::chrono::steady_clock::now() < clean_drain_deadline_) {
          stopBase();
          break;
        }
        RCLCPP_INFO(this->get_logger(),
                    "[WaitForClean] Got Clean_up! → resolving destination gesture");
        received_clean_command_ = false;
        stopBase();

        // Clean_up! is the end marker for the second gesture. Resolve only if
        // at least one settled ray was captured after the protocol boundary.
        if (destination_pointing_history_.empty()) {
          RCLCPP_WARN(
            this->get_logger(),
            "[WaitForClean] No settled destination gesture was captured before "
            "Clean_up!; continuing with pickup-only scoring");
          // Target recognition and pickup are independently scoreable.  A
          // missing destination must not discard the already confirmed target
          // or prevent the robot from attempting the grasp.
          dest_valid_ = false;
          placement_ready_ = false;
          enableVision(false);
          changeStep(MoveToObject);
        } else if (selectDestinationByPointingRay()) {
          enableVision(false);
          changeStep(MoveToObject);
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "[WaitForClean] Settled pre-Clean destination gesture "
                      "was insufficient");
          // Clean_up! closes this task's second gesture. Continuing to watch
          // after it consumes the next task's Pick/gesture, as seen in the
          // 22:48 run. Never recover across that protocol boundary.
          // Still preserve the independently scoreable pickup phase.  Do not
          // invent a placement coordinate or alter either recognition path.
          dest_valid_ = false;
          placement_ready_ = false;
          enableVision(false);
          RCLCPP_WARN(
            this->get_logger(),
            "[WaitForClean] Destination unresolved; continuing with "
            "pickup-only scoring");
          changeStep(MoveToObject);
        }
      } else if (elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
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
          if (!target_valid_ ||
              !isReachableGraspHeight(target_position_.z)) {
            RCLCPP_ERROR(this->get_logger(),
                         "[MoveToObj] Target height %.3fm is invalid or unreachable; "
                         "refusing to navigate to an unverified target",
                         target_position_.z);
            giveUp();
            changeStep(WaitForResult);
            break;
          }
          // The observation wrist pose projects the open hand across the
          // close-range camera view (recorded floor-target occlusion).
          // Tuck the wrist during approach; extend only for pregrasp.
          armInitial();
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
          headLookAtTarget();
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
              // Image-centre proximity alone can switch to another instance
              // of the same class. Refine only near the selected world point.
              if (!o.has_3d_position ||
                  !isReachableGraspHeight(o.position_3d.z) ||
                  std::hypot(o.position_3d.x-target_position_.x,
                             o.position_3d.y-target_position_.y) > 0.20 ||
                  std::abs(o.position_3d.z-target_position_.z) > 0.10) continue;
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
          // Extend above the remembered object height while it is still safe,
          // then align the actual fingertip midpoint before lowering.  The
          // close-range target is frozen here: the arm can occlude the object
          // and corrupt a new depth estimate with its own surface.
          armGraspReadyForTarget(); headLookAtTarget();
          enableVision(false);
          RCLCPP_INFO(this->get_logger(),
                      "[Grasp] Height-aware pregrasp target_z=%.3fm ready_tip_z=%.3fm",
                      target_position_.z,
                      target_position_.z + GRIPPER_PREGRASP_CLEARANCE);
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
          const double surface_forward =
            std::cos(robot.yaw) * world_dx +
            std::sin(robot.yaw) * world_dy;
          // Positive motion advances the base.  Requiring the trigger
          // midpoint to finish slightly beyond the visible front surface
          // places the object inside the open fingers before they close.
          // Low floor objects are easily pushed by the descending fingers.
          // Remove the extra 1 cm insertion for this height band only;
          // preserve the validated elevated-object grasp geometry.
          const double insertion_depth = target_position_.z < 0.15
            ? 0.0 : GRIPPER_TARGET_INSERTION_DEPTH;
          double forward = surface_forward + insertion_depth;
          double lateral = -std::sin(robot.yaw) * world_dx +
                            std::cos(robot.yaw) * world_dy;

          const bool aligned = graspHorizontalAligned(target_position_, gripper, robot.yaw);
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
                        "[Grasp] Finger trigger aligned surface_forward=%.3fm "
                        "servo_forward=%.3fm lateral=%.3fm total=%.3fm; lowering",
                        surface_forward, forward, lateral, error);
            armGraspAtTargetHeight();
            sub_step_ = 2;
            enterTimedState();
            break;
          }

          if (elapsed() >= GRIPPER_ALIGN_TIMEOUT) {
            stopBase();
            enableVision(false);
            RCLCPP_ERROR(this->get_logger(),
                         "[Grasp] Precise fingertip alignment failed: "
                         "surface_forward=%.3fm servo_forward=%.3fm "
                         "lateral=%.3fm total=%.3fm; "
                         "refusing blind grasp",
                         surface_forward, forward, lateral, error);
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
            // The 3 mm elevated insertion tolerance belongs to the moving
            // pregrasp servo. Applying it as a new abort condition after
            // descent regressed successful grasps. Restore the established
            // height-checked closure and retain XY as diagnostic evidence.
            const auto robot = getRobotPose();
            if (robot.valid) {
              const double dx = target_position_.x - gripper.x;
              const double dy = target_position_.y - gripper.y;
              RCLCPP_INFO(this->get_logger(),
                "[Grasp] Post-lowering diagnostic: surface_forward=%.4fm lateral=%.4fm",
                std::cos(robot.yaw)*dx + std::sin(robot.yaw)*dy,
                -std::sin(robot.yaw)*dx + std::cos(robot.yaw)*dy);
            }
            const double desired_tip_z = std::max(
              GRIPPER_DOWNWARD_BASE_HEIGHT,
              target_position_.z + GRIPPER_CONTACT_HEIGHT_OFFSET);
            const double height_error = gripper.z - desired_tip_z;
            if (std::abs(height_error) > GRIPPER_HEIGHT_TOLERANCE) {
              stopBase();
              RCLCPP_ERROR(this->get_logger(),
                           "[Grasp] Fingertip height does not match target: "
                           "tip_z=%.3fm desired_z=%.3fm target_z=%.3fm "
                           "error=%.3fm",
                           gripper.z, desired_tip_z, target_position_.z,
                           height_error);
              giveUp();
              changeStep(WaitForResult);
              break;
            }
            RCLCPP_INFO(this->get_logger(),
                        "[Grasp] Target-height fingertip verified: "
                        "tip_z=%.3fm desired_z=%.3fm target_z=%.3fm; closing",
                        gripper.z, desired_tip_z, target_position_.z);
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
      if (sub_step_ == 0) {
        // Always report the independently completed grasp before deciding
        // whether a placement phase is available.
        sendMessage(MSG_OBJECT_GRASPED);
        if (dest_valid_) {
          changeStep(MoveToDestination);
        } else {
          stopBase();
          RCLCPP_WARN(
            this->get_logger(),
            "[SendGrasped] Pickup completed and reported, but no valid "
            "destination is available; preserving grasp before Give_up");
          sub_step_ = 1;
          enterTimedState();
        }
      } else if (sub_step_ == 1 && elapsed() >= 1.0) {
        // Give the moderator time to evaluate Object_grasped while the object
        // remains in the closed gripper and the arm remains in carry pose.
        // Sending Give_up directly avoids emergencyStop retracting the arm
        // before that independently scoreable event has been evaluated.
        sendMessage(MSG_GIVE_UP);
        changeStep(WaitForResult);
      }
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
          const double current_lift =
            getJointPosition("arm_lift_joint", ARM_RELEASE_READY[0]);
          // For a drop-in container the configured point is the judge-trigger
          // centre, not a height the fingers must physically reach. Once XY is
          // centred, opening at the lowest reachable pose above that centre
          // lets gravity place the object in the container. Repeating a
          // clamped zero-lift command can never improve the pose and must not
          // turn a valid arrival into Give_up.
          const bool reachable_drop_pose =
            placement_profile_.drop_in && height_error < 0.0 &&
            current_lift <= 0.005;
          if (reachable_drop_pose) {
            RCLCPP_INFO(
              this->get_logger(),
              "[Release] Drop-in target centre is below arm reach; "
              "opening at lowest reachable height %.3fm above %.3fm",
              gripper.z, placement_position_.z);
            gripperOpen();
            sub_step_ = 3;
            enterTimedState();
            break;
          }
          if (std::abs(height_error) > RELEASE_HEIGHT_TOLERANCE &&
              release_height_adjustments_ < 3) {
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
      if (!is_finished_ && elapsed() >= 5.0 * (sub_step_ + 1)) {
        ++sub_step_;
        RCLCPP_WARN(this->get_logger(),
          "[WaitResult] No moderator result after %.1fs; "
          "moderator-topic subscribers=%zu. Not declaring success or replaying commands.",
          elapsed(), pub_msg_->get_subscription_count());
      }
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

#ifndef CLEANUP_CONTROLLER_TEST
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractiveCleanupSample>();
  int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
#endif
