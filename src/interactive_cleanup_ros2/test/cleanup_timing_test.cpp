#include <gtest/gtest.h>
#define CLEANUP_CONTROLLER_TEST
#include "../src/interactive_cleanup_sample.cpp"

// Exercise actual callbacks/state transitions without spinning an executor or
// connecting to the simulator. All command publishers use test-only topics.
struct CleanupTimingAccess
{
  static void readinessHandshake()
  {
    auto n=make(); n->step_=n->Ready;
    n->arm_joint_trajectory_.points.resize(1);
    n->pub_arm_trajectory_=n->create_publisher<trajectory_msgs::msg::JointTrajectory>("/cleanup_timing_test/arm",10);
    n->pub_msg_=n->create_publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>("/cleanup_timing_test/moderator",10);
    n->pub_msg_human_=n->create_publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>("/cleanup_timing_test/human",10);
    n->pub_hsr_msg_=n->create_publisher<std_msgs::msg::String>("/cleanup_timing_test/hsr",10);
    n->handleIncomingMessage("Are_you_ready?", "", "test");
    EXPECT_TRUE(n->is_started_);
    EXPECT_FALSE(n->ready_ack_sent_);
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->Ready);
    n->vision_ready_=true;
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->WaitForPickCommand);
    EXPECT_TRUE(n->ready_ack_sent_);
    n->reset(); EXPECT_TRUE(n->vision_ready_);
  }
  static void futureCaptureDuringMotion()
  {
    auto n=make(); n->step_=n->ObserveTarget;
    const auto start=std::chrono::steady_clock::now();
    n->moveHead(1.5,0,0.3); // joint-speed limit extends the requested duration
    EXPECT_GE(std::chrono::duration<double>(n->head_motion_ready_time_-start).count(),1.5);
    auto p=gesture(n->head_observation_not_before_ns_+2000000000LL);
    // Reproduce source stamps ahead of the host and an expired caller timer.
    n->pointing_camera_ready_time_={};
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    EXPECT_FALSE(n->has_person_observation_);
    EXPECT_FALSE(n->has_last_valid_pointing_);
    auto objects=std::make_shared<DetectedObjectArray>();
    objects->header.stamp=p.header.stamp;
    objects->received_steady_ns=p.received_steady_ns;
    n->objectsCallback(objects);
    EXPECT_TRUE(n->obs_objects_.empty());
    n->head_motion_ready_time_={};
    p.received_steady_ns=n->head_observation_not_before_ns_+1;
    objects->received_steady_ns=p.received_steady_ns;
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    n->objectsCallback(objects);
    EXPECT_TRUE(n->has_person_observation_);
    EXPECT_TRUE(n->has_last_valid_pointing_);
    EXPECT_EQ(n->obs_objects_.size(),1u);
    n->reset();
    EXPECT_EQ(n->head_motion_ready_time_.time_since_epoch().count(),0);
  }
  static void visibleDuringMotionDoesNotRestartSearch()
  {
    auto n=make();
    n->moveHead(0,0,1);
    auto p=gesture(n->head_observation_not_before_ns_-1);
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    EXPECT_FALSE(n->has_person_observation_);
    EXPECT_FALSE(n->has_last_valid_pointing_);
    n->searchAvatarWithHead();
    EXPECT_EQ(n->avatar_search_pose_index_,0);
    // Motion finishes before a usable settled frame arrives. Visibility must
    // still prevent restarting the scan and throwing away that next frame.
    n->head_motion_ready_time_={}; n->pointing_camera_ready_time_={};
    n->searchAvatarWithHead();
    EXPECT_EQ(n->avatar_search_pose_index_,0);
    n->last_person_seen_time_=std::chrono::steady_clock::now()-std::chrono::seconds(30);
    n->searchAvatarWithHead();
    EXPECT_EQ(n->avatar_search_pose_index_,1);
    n->reset();
    EXPECT_EQ(n->last_person_seen_time_.time_since_epoch().count(),0);
  }
  static void phaseIgnoresSourceClock()
  {
    auto n=make();
    n->handleIncomingMessage("Pick_it_up!", "", "test");
    n->step_=n->WaitForCleanCommand;
    n->handleIncomingMessage("Clean_up!", "", "test");
    for (int64_t source : {1000000000LL, 1793930619000000000LL}) {
      auto p=gesture(source); p.plane_valid=true;
      p.received_steady_ns=n->destination_window_.begin_ns-1;
      n->pointingCallback(std::make_shared<PointingDirection>(p));
      EXPECT_TRUE(n->destination_pointing_history_.empty());
      p.received_steady_ns=n->destination_window_.begin_ns;
      n->pointingCallback(std::make_shared<PointingDirection>(p));
      EXPECT_EQ(n->destination_pointing_history_.size(),1u);
      n->destination_pointing_history_.clear();
      p.received_steady_ns=n->destination_window_.end_ns+1;
      n->pointingCallback(std::make_shared<PointingDirection>(p));
      EXPECT_TRUE(n->destination_pointing_history_.empty());
      p.received_steady_ns=0;
      n->pointingCallback(std::make_shared<PointingDirection>(p));
      EXPECT_TRUE(n->destination_pointing_history_.empty());
    }
  }
  static void captureBarrier()
  {
    auto n=make(); n->step_=n->ObserveTarget;
    n->moveHead(0.2,-0.2,0.5);
    const auto boundary=n->head_observation_not_before_ns_;
    // Arrives after motion, but was captured during it.
    n->pointing_camera_ready_time_={};
    auto old=gesture(boundary-1);
    old.received_steady_ns=boundary-1;
    n->pointingCallback(std::make_shared<PointingDirection>(old));
    EXPECT_FALSE(n->has_last_valid_pointing_);
    EXPECT_FALSE(n->has_person_observation_);
    auto objects=std::make_shared<DetectedObjectArray>();
    objects->header.stamp=old.header.stamp;
    objects->received_steady_ns=old.received_steady_ns;
    n->objectsCallback(objects);
    EXPECT_TRUE(n->obs_objects_.empty());
    n->head_motion_ready_time_ = {};
    auto fresh=gesture(boundary+1);
    fresh.received_steady_ns=boundary+1;
    n->pointingCallback(std::make_shared<PointingDirection>(fresh));
    EXPECT_TRUE(n->has_last_valid_pointing_);
    objects->header.stamp=fresh.header.stamp;
    objects->received_steady_ns=fresh.received_steady_ns;
    n->objectsCallback(objects);
    EXPECT_EQ(n->obs_objects_.size(),1u);
  }
  static void revisableHypotheses(bool conflict)
  {
    auto n=make();
    const auto now=InteractiveCleanupSample::ingressNowNs();
    auto good=gesture(now-1000000000LL);
    good.origin.x=0; good.origin.z=0.1;
    good.direction.x=1; good.direction.y=0; good.direction.z=0;
    good.depth_direction_valid=true;
    auto wrong=good; wrong.direction.x=0; wrong.direction.y=1;
    n->has_locked_pointing_=true; n->locked_pointing_=wrong;
    n->pickup_ray_hypotheses_={good};
    DetectedObject target; target.class_name="bear_doll";
    target.confidence=0.9; target.has_3d_position=true;
    target.position_3d.x=2; target.position_3d.z=0.1;
    DetectedObjectArray frame; frame.objects={target};
    if (conflict) {
      auto other=target; other.position_3d.x=0; other.position_3d.y=2;
      frame.objects.push_back(other);
    }
    for(int i=0;i<6;++i) n->obs_objects_.push_back(frame);
    EXPECT_EQ(n->selectStablePointedTarget(),!conflict);
    EXPECT_EQ(n->target_valid_,!conflict);
    if(!conflict) { EXPECT_DOUBLE_EQ(n->locked_pointing_.direction.x,1); }
    n->reset(); EXPECT_TRUE(n->pickup_ray_hypotheses_.empty());
  }
  static void recordedDestinationWithTransitionFrames()
  {
    auto n=make();
    // Actual capture-time observations from 20261027-215826 task 3,
    // ending at Clean_up!; three pose transitions and six measured rays.
    { auto p=gesture(1793109696137000000LL);
      p.confidence=0.8449745178222656;
      p.plane_valid=true;
      p.depth_direction_valid=false;
      p.pose_direction_valid=true;
      p.origin.x=0.031126706764933093;
      p.origin.y=1.812023916640691;
      p.origin.z=0.7831651577327235;
      p.direction.x=-0.10854078221840201;
      p.direction.y=0.46669799517403193;
      p.direction.z=-0.8777310977149875;
      p.pose_direction.x=-0.10854078221840201;
      p.pose_direction.y=0.46669799517403193;
      p.pose_direction.z=-0.8777310977149875;
      p.plane_origin.x=-0.05495148416927364;
      p.plane_origin.y=-0.08036219529527112;
      p.plane_origin.z=1.000203810208338;
      p.plane_normal.x=-0.9766244680860603;
      p.plane_normal.y=0.06781721338017177;
      p.plane_normal.z=0.20397419911588802;
      p.plane_forward.x=-0.21015861367090147;
      p.plane_forward.y=-0.10195830735732245;
      p.plane_forward.z=-0.9723362899021895;
      p.camera_forward.x=0.35337349829759607;
      p.camera_forward.y=0.9194683838176968;
      p.camera_forward.z=0.17235156471756075;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109697363000000LL);
      p.confidence=0.8910771012306213;
      p.plane_valid=true;
      p.depth_direction_valid=false;
      p.pose_direction_valid=true;
      p.origin.x=-0.3442681046521146;
      p.origin.y=1.9808634054013643;
      p.origin.z=1.3743974922758653;
      p.direction.x=-0.8157850509233259;
      p.direction.y=-0.44085880648650666;
      p.direction.z=0.3743504553667855;
      p.pose_direction.x=-0.8157850509233259;
      p.pose_direction.y=-0.44085880648650666;
      p.pose_direction.z=0.3743504553667855;
      p.plane_origin.x=-0.05445786808309934;
      p.plane_origin.y=-0.07537364428311318;
      p.plane_origin.z=1.0031229780647424;
      p.plane_normal.x=0.5982039435588077;
      p.plane_normal.y=-0.05997292751554565;
      p.plane_normal.z=0.7990965460292679;
      p.plane_forward.x=-0.7894793598618335;
      p.plane_forward.y=-0.21506827049302557;
      p.plane_forward.z=0.57486344411459;
      p.camera_forward.x=0.35415028901905954;
      p.camera_forward.y=0.9235823943841792;
      p.camera_forward.z=0.14687795468109932;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109697646000000LL);
      p.confidence=0.512198269367218;
      p.plane_valid=true;
      p.depth_direction_valid=false;
      p.pose_direction_valid=true;
      p.origin.x=-0.14649142013042993;
      p.origin.y=2.059753394741935;
      p.origin.z=1.2496011318490752;
      p.direction.x=-0.9385091185458246;
      p.direction.y=0.3121485397168502;
      p.direction.z=0.14752600977108243;
      p.pose_direction.x=-0.9385091185458246;
      p.pose_direction.y=0.3121485397168502;
      p.pose_direction.z=0.14752600977108243;
      p.plane_origin.x=-0.05383196039843891;
      p.plane_origin.y=-0.07265473745673887;
      p.plane_origin.z=1.0044709968862786;
      p.plane_normal.x=0.361213451679228;
      p.plane_normal.y=-0.0909859423309541;
      p.plane_normal.z=0.9280336204169148;
      p.plane_forward.x=-0.9314852516795387;
      p.plane_forward.y=-0.08123780670881738;
      p.plane_forward.z=0.3545922230741211;
      p.camera_forward.x=0.3548234081543374;
      p.camera_forward.y=0.925177713769379;
      p.camera_forward.z=0.1347091198479362;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109698960000000LL);
      p.confidence=0.635190486907959;
      p.plane_valid=true;
      p.depth_direction_valid=true;
      p.pose_direction_valid=true;
      p.origin.x=-0.156717535600949;
      p.origin.y=2.0422578396708184;
      p.origin.z=1.2195026495421353;
      p.direction.x=-0.7898467780346176;
      p.direction.y=-0.5675876864427298;
      p.direction.z=-0.2323494898357281;
      p.pose_direction.x=-0.9260338696988963;
      p.pose_direction.y=0.18389468055932157;
      p.pose_direction.z=-0.32961192125357436;
      p.plane_origin.x=-0.05560201852024392;
      p.plane_origin.y=-0.07874220173996252;
      p.plane_origin.z=1.000568153997248;
      p.plane_normal.x=-0.20678542297776087;
      p.plane_normal.y=-0.11020673241032114;
      p.plane_normal.z=0.9721595882237384;
      p.plane_forward.x=-0.9772389839129366;
      p.plane_forward.y=-0.024841339840722346;
      p.plane_forward.z=-0.2106819312511852;
      p.camera_forward.x=0.35262733767367466;
      p.camera_forward.y=0.920331053744562;
      p.camera_forward.z=0.16924748813084492;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109699240000000LL);
      p.confidence=0.6695912480354309;
      p.plane_valid=true;
      p.depth_direction_valid=true;
      p.pose_direction_valid=true;
      p.origin.x=-0.16584633244728014;
      p.origin.y=2.0345157924605157;
      p.origin.z=1.2283231723896295;
      p.direction.x=-0.8084740480807953;
      p.direction.y=-0.5197824789834591;
      p.direction.z=-0.27603602685457895;
      p.pose_direction.x=-0.9277494789766355;
      p.pose_direction.y=0.1148655795341568;
      p.pose_direction.z=-0.35508703566430533;
      p.plane_origin.x=-0.0537600313160956;
      p.plane_origin.y=-0.07482313369210754;
      p.plane_origin.z=1.0037627038391175;
      p.plane_normal.x=-0.25350893022460447;
      p.plane_normal.y=-0.1157139175688001;
      p.plane_normal.z=0.960387167540913;
      p.plane_forward.x=-0.9658928410359975;
      p.plane_forward.y=-0.023876158114533634;
      p.plane_forward.z=-0.2578389976498884;
      p.camera_forward.x=0.354134333886864;
      p.camera_forward.y=0.9244857881236321;
      p.camera_forward.z=0.14112016553254958;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109699352000000LL);
      p.confidence=0.6995645761489868;
      p.plane_valid=true;
      p.depth_direction_valid=true;
      p.pose_direction_valid=true;
      p.origin.x=-0.15548993900698999;
      p.origin.y=2.048256755336261;
      p.origin.z=1.236389064441258;
      p.direction.x=-0.8711542661605403;
      p.direction.y=-0.45355464339549045;
      p.direction.z=-0.1880915468719423;
      p.pose_direction.x=-0.941846945499245;
      p.pose_direction.y=0.09751970481927466;
      p.pose_direction.z=-0.32158084275296;
      p.plane_origin.x=-0.05319275884033488;
      p.plane_origin.y=-0.07341794635531584;
      p.plane_origin.z=1.0046449318705695;
      p.plane_normal.x=-0.1523047716639748;
      p.plane_normal.y=-0.11456835097822296;
      p.plane_normal.z=0.9816706929935901;
      p.plane_forward.x=-0.9871733460194642;
      p.plane_forward.y=-0.030479278755380344;
      p.plane_forward.z=-0.1567156612316942;
      p.camera_forward.x=0.3544611418450344;
      p.camera_forward.y=0.92555401875023;
      p.camera_forward.z=0.13306786726033226;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109699637000000LL);
      p.confidence=0.7239850759506226;
      p.plane_valid=true;
      p.depth_direction_valid=true;
      p.pose_direction_valid=true;
      p.origin.x=-0.15403817034983167;
      p.origin.y=2.062811161594104;
      p.origin.z=1.2354548311539717;
      p.direction.x=-0.8687117384095763;
      p.direction.y=-0.4441774951198352;
      p.direction.z=-0.21919459021262563;
      p.pose_direction.x=-0.9373343703929177;
      p.pose_direction.y=0.09406466949193767;
      p.pose_direction.z=-0.3354938390395345;
      p.plane_origin.x=-0.052504919254326106;
      p.plane_origin.y=-0.07102939512565026;
      p.plane_origin.z=1.0051874206798346;
      p.plane_normal.x=-0.1877576795149831;
      p.plane_normal.y=-0.11420801861822254;
      p.plane_normal.z=0.9755529623072491;
      p.plane_forward.x=-0.9810779931235499;
      p.plane_forward.y=-0.02597776676872521;
      p.plane_forward.z=-0.19186226059957084;
      p.camera_forward.x=0.3545053489936615;
      p.camera_forward.y=0.9262450662666617;
      p.camera_forward.z=0.12804700211855735;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109701033000000LL);
      p.confidence=0.7522774338722229;
      p.plane_valid=true;
      p.depth_direction_valid=true;
      p.pose_direction_valid=true;
      p.origin.x=-0.15926441230179333;
      p.origin.y=2.071045452679;
      p.origin.z=1.2389845143372513;
      p.direction.x=-0.8107510201101831;
      p.direction.y=-0.47635848654165835;
      p.direction.z=-0.3402431126268957;
      p.pose_direction.x=-0.9120267294447135;
      p.pose_direction.y=0.0789389617604384;
      p.pose_direction.z=-0.40246227785292293;
      p.plane_origin.x=-0.05391955988308847;
      p.plane_origin.y=-0.07450387904007093;
      p.plane_origin.z=1.0035333320226798;
      p.plane_normal.x=-0.3240132517979658;
      p.plane_normal.y=-0.11890472710606198;
      p.plane_normal.z=0.9385505199674342;
      p.plane_forward.x=-0.94479575773565;
      p.plane_forward.y=-0.010449871870107291;
      p.plane_forward.z=-0.327493169917507;
      p.camera_forward.x=0.3538115384664921;
      p.camera_forward.y=0.9242915923589237;
      p.camera_forward.z=0.14318675756710011;
      n->destination_pointing_history_.push_back(p); }
    { auto p=gesture(1793109701316000000LL);
      p.confidence=0.7673885226249695;
      p.plane_valid=true;
      p.depth_direction_valid=true;
      p.pose_direction_valid=true;
      p.origin.x=-0.169316659915362;
      p.origin.y=2.0665801901348133;
      p.origin.z=1.2341790697305024;
      p.direction.x=-0.8098034553686179;
      p.direction.y=-0.4958457844342178;
      p.direction.z=-0.31361652018326786;
      p.pose_direction.x=-0.9145435151982633;
      p.pose_direction.y=0.10000134560038335;
      p.pose_direction.z=-0.3919309756665284;
      p.plane_origin.x=-0.05476522219334359;
      p.plane_origin.y=-0.07422298792201804;
      p.plane_origin.z=1.0035947624022907;
      p.plane_normal.x=-0.29501076558979794;
      p.plane_normal.y=-0.11791448680117511;
      p.plane_normal.z=0.9481902878581583;
      p.plane_forward.x=-0.9540158797532111;
      p.plane_forward.y=-0.018825311183889675;
      p.plane_forward.z=-0.2991643508801414;
      p.camera_forward.x=0.35411387474495104;
      p.camera_forward.y=0.924261310805954;
      p.camera_forward.z=0.1426337725097988;
      n->destination_pointing_history_.push_back(p); }
    ASSERT_TRUE(n->selectDestinationByPointingRay());
    EXPECT_EQ(n->dest_class_, "trash_box_for_burnable");
  }
  static void retainedArmPickup(int variant)
  {
    auto n = make();
    const int64_t stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()-4000000000LL;
    auto arm = gesture(stamp);
    arm.arm_ray_valid = true;
    arm.arm_ray_origin.x=2.9; arm.arm_ray_origin.y=0.7; arm.arm_ray_origin.z=0.9;
    arm.arm_ray_direction.x=1.84-2.9;
    arm.arm_ray_direction.y=0.85-0.7;
    arm.arm_ray_direction.z=0.06-0.9;
    // Capture-time world geometry is useful even when delivered during a
    // tracking movement. It must not create a premature preview lock.
    n->pointing_camera_ready_time_=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    n->pointingCallback(std::make_shared<PointingDirection>(arm));
    ASSERT_TRUE(n->has_retained_arm_pointing_);
    EXPECT_FALSE(n->lockRecentPointing());
    n->invalidatePickupPointingForHeadMotion(std::chrono::steady_clock::now(),0.3);
    EXPECT_TRUE(n->has_retained_arm_pointing_);
    auto crop = gesture(stamp+1000000000LL);
    crop.point_pixel_x=crop.wrist_pixel_x+7;
    crop.point_pixel_y=crop.wrist_pixel_y;
    n->last_raw_pointing_time_ = {};
    n->pointingCallback(std::make_shared<PointingDirection>(crop));
    EXPECT_FALSE(n->hasRecentRawPointing()); // Rejected crop cannot suppress search.
    EXPECT_EQ(n->retained_arm_pointing_.header.stamp, arm.header.stamp);
    n->handleIncomingMessage("Pick_it_up!", "", "test");
    n->step_=n->ObserveTarget;
    // Later destination gestures and out-of-order frames cannot overwrite it.
    auto later=arm;
    later.header.stamp=rclcpp::Time(n->destination_window_.begin_ns+1);
    later.arm_ray_direction.x=99;
    n->pointingCallback(std::make_shared<PointingDirection>(later));
    auto older=later; older.header.stamp=rclcpp::Time(stamp-1);
    n->pointingCallback(std::make_shared<PointingDirection>(older));
    EXPECT_EQ(n->retained_arm_pointing_.header.stamp, arm.header.stamp);
    if (variant==1) n->retained_arm_time_-=std::chrono::seconds(21);
    if (variant==2) n->reset();
    EXPECT_FALSE(n->lockRecentPointing()); // Memory alone must not stop search.
    EXPECT_FALSE(n->has_locked_pointing_);
    if (variant!=0) return;
    EXPECT_FALSE(n->target_valid_);
    DetectedObject target;
    target.class_name="soysauce"; target.confidence=0.94; target.has_3d_position=true;
    target.position_3d.x=1.84; target.position_3d.y=0.85; target.position_3d.z=0.06;
    auto other=target; other.class_name="rabbit_doll";
    other.position_3d.x=4.01; other.position_3d.y=1.95;
    DetectedObjectArray frame; frame.objects={target,other};
    n->obs_objects_.push_back(frame);
    EXPECT_FALSE(n->lockRecentPointing());
    EXPECT_FALSE(n->has_locked_pointing_);
    for(int i=1;i<6;++i) n->obs_objects_.push_back(frame);
    // The exact old walking-arm ray from 20261027-215826 cannot freeze
    // observation even with persistent objects in the camera image.
    auto saved=n->retained_arm_pointing_;
    n->retained_arm_pointing_.origin.x=2.354713;
    n->retained_arm_pointing_.origin.y=0.202802;
    n->retained_arm_pointing_.origin.z=1.277257;
    n->retained_arm_pointing_.direction.x=0.396377;
    n->retained_arm_pointing_.direction.y=0.893876;
    n->retained_arm_pointing_.direction.z=0.209453;
    EXPECT_FALSE(n->lockRecentPointing());
    EXPECT_FALSE(n->has_locked_pointing_);
    n->retained_arm_pointing_=saved;
    ASSERT_TRUE(n->lockRecentPointing());
    EXPECT_TRUE(n->locked_historical_arm_);
    EXPECT_FALSE(n->locked_pointing_.depth_direction_valid);
    EXPECT_FALSE(n->aimHeadAlongPointing(n->locked_pointing_));
    EXPECT_EQ(n->target_class_,"soysauce");
  }
  static void transientHandDoesNotRestartSearch()
  {
    auto n = make();
    n->close_gesture_search_active_ = true;
    n->close_gesture_search_index_ = 3;
    auto p = gesture(1000000000);
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    EXPECT_TRUE(n->close_gesture_search_active_);
    EXPECT_EQ(n->close_gesture_search_index_, 3);
    EXPECT_FALSE(n->lockRecentPointing());
    n->last_close_gesture_search_time_ = std::chrono::steady_clock::now();
    n->last_person_pixel_x_ = 450;
    EXPECT_FALSE(n->trackAvatarWithHead());
  }
  static void descentKeepsEstablishedClosure()
  {
    auto n = make();
    n->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(n->get_clock());
    n->pub_gripper_trajectory_ = n->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/cleanup_timing_test/gripper", 10);
    n->step_ = n->GraspObject; n->sub_step_ = 2;
    n->target_valid_ = true;
    n->target_position_.x = 0.47; n->target_position_.y = 2.86;
    n->target_position_.z = 0.545;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom"; tf.transform.rotation.w = 1.0;
    tf.child_frame_id = "base_footprint";
    n->tf_buffer_->setTransform(tf, "test", true);
    // Five millimetres of insertion error exceeds the moving servo's 3 mm
    // tolerance, but must not cause a new abort after descent.
    tf.transform.translation.x = 0.475;
    tf.transform.translation.y = 2.86;
    tf.transform.translation.z = 0.530;
    for (const auto &frame : {"hand_l_finger_tip_frame", "hand_r_finger_tip_frame",
                             "hand_l_distal_link", "hand_r_distal_link"}) {
      tf.child_frame_id = frame;
      n->tf_buffer_->setTransform(tf, "test", true);
    }
    n->state_timer_initialized_ = true;
    n->state_enter_time_ = std::chrono::steady_clock::now()-std::chrono::seconds(3);
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->GraspObject);
    EXPECT_EQ(n->sub_step_, 3); // Close, then lift; no Give_up transition.
  }
  static void shortFallbackLock()
  {
    auto n = make();
    n->step_ = n->ObserveTarget;
    auto p = gesture(1000000000);
    p.point_pixel_x = p.wrist_pixel_x + 6.8;
    p.point_pixel_y = p.wrist_pixel_y;
    n->has_last_valid_pointing_ = true;
    n->last_valid_pointing_ = p;
    n->last_valid_pointing_time_ = std::chrono::steady_clock::now();
    EXPECT_FALSE(n->lockRecentPointing());
    EXPECT_FALSE(n->has_locked_pointing_);
    // Above the image-length boundary still needs temporal confirmation.
    p.point_pixel_x = p.wrist_pixel_x + 8.1;
    n->has_last_valid_pointing_ = true;
    n->last_valid_pointing_ = p;
    n->pickup_pointing_history_ = {p};
    EXPECT_FALSE(n->lockRecentPointing());
    auto q = p; q.header.stamp = rclcpp::Time(int64_t(1300000000));
    auto r = p; r.header.stamp = rclcpp::Time(int64_t(1700000000));
    n->last_valid_pointing_ = r;
    n->pickup_pointing_history_ = {p,q,r};
    EXPECT_TRUE(n->lockRecentPointing());
  }
  static void briefMeasuredGesture(int variant)
  {
    auto n = make();
    n->step_=n->ObserveTarget;
    n->pointing_camera_ready_time_=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    auto p=gesture(1000000000);
    p.depth_direction_valid=true; p.plane_valid=true;
    p.depth_direction_samples=9; p.depth_direction_fit_error=0.012;
    p.confidence=0.95;
    if (variant==1) p.confidence=0.5;
    if (variant==2) p.depth_direction_valid=false;
    if (variant==3) p.depth_direction_fit_error=0.03;
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    EXPECT_FALSE(n->has_last_valid_pointing_);
    EXPECT_TRUE(n->pickup_pointing_history_.empty());
    EXPECT_FALSE(n->lockRecentPointing());
    if (variant != 0) return;
    n->pointing_camera_ready_time_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    ASSERT_TRUE(n->has_last_valid_pointing_);
    EXPECT_FALSE(n->lockRecentPointing()); // High confidence is not temporal support.
    n->invalidatePickupPointingForHeadMotion(std::chrono::steady_clock::now(),0.3);
    EXPECT_TRUE(n->pickup_pointing_history_.empty());
    EXPECT_FALSE(n->has_last_valid_pointing_);
    EXPECT_FALSE(n->target_valid_); // A gesture alone never selects an object.
  }
  static void elevatedInsertionTolerance()
  {
    EXPECT_DOUBLE_EQ(InteractiveCleanupSample::graspForwardTolerance(0.079), 0.010);
    EXPECT_DOUBLE_EQ(InteractiveCleanupSample::graspForwardTolerance(0.540), 0.003);
    // Recorded elevated grasp stopped with 9 mm still to travel. It must
    // continue, whereas the successful floor grasp keeps its 9 mm window.
    EXPECT_GT(0.009, InteractiveCleanupSample::graspForwardTolerance(0.540));
    EXPECT_LE(0.009, InteractiveCleanupSample::graspForwardTolerance(0.079));
    EXPECT_LE(0.002, InteractiveCleanupSample::graspForwardTolerance(0.540));
  }
  static void completionWaitsForModerator()
  {
    auto n = make();
    using Msg = interactive_cleanup_msgs::msg::InteractiveCleanupMsg;
    n->pub_msg_ = n->create_publisher<Msg>("/cleanup_timing_test/moderator", 10);
    n->pub_msg_human_ = n->create_publisher<Msg>("/cleanup_timing_test/human", 10);
    n->pub_hsr_msg_ = n->create_publisher<std_msgs::msg::String>("/cleanup_timing_test/hsr", 10);
    n->changeStep(n->SendTaskFinished);
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->WaitForResult);
    EXPECT_FALSE(n->is_finished_);
    n->state_enter_time_=std::chrono::steady_clock::now()-std::chrono::seconds(6);
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->WaitForResult);
    EXPECT_EQ(n->sub_step_, 1); // Diagnostic only, no automatic success.
    EXPECT_FALSE(n->is_finished_);
    n->handleIncomingMessage("Task_succeeded", "", "test");
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->Initialize);
  }
  static void trackingAcquisitionWindow()
  {
    auto n = make();
    auto now = std::chrono::steady_clock::now();
    n->has_person_observation_=true;
    n->last_person_observation_time_=now;
    n->last_person_pixel_x_=450; n->last_person_pixel_y_=240;
    n->pickup_pointing_history_.push_back(gesture(1000000000));
    n->pointing_camera_ready_time_=now+std::chrono::milliseconds(400);
    EXPECT_FALSE(n->trackAvatarWithHead());
    n->pointing_camera_ready_time_=now-std::chrono::milliseconds(100);
    EXPECT_FALSE(n->trackAvatarWithHead());
    n->searchGestureAroundVisibleAvatar();
    EXPECT_EQ(n->pickup_pointing_history_.size(),1u);
    n->pointing_camera_ready_time_=now-std::chrono::milliseconds(800);
    EXPECT_TRUE(n->trackAvatarWithHead());
    EXPECT_TRUE(n->pickup_pointing_history_.empty());
    // At an image edge, follow again once settled without the extra hold.
    n->last_avatar_track_time_=now-std::chrono::seconds(2);
    n->last_person_pixel_x_=600;
    n->pointing_camera_ready_time_=now-std::chrono::milliseconds(100);
    EXPECT_TRUE(n->trackAvatarWithHead());
  }
  static void loweredHorizontalAlignment()
  {
    geometry_msgs::msg::Point target, gripper;
    target.x=1.565; target.y=-1.463; target.z=0.076;
    gripper=target; gripper.x-=0.008;
    EXPECT_TRUE(InteractiveCleanupSample::graspHorizontalAligned(target,gripper,0.0));
    // Correct height alone cannot conceal a 3 cm displacement during descent.
    gripper.y+=0.03;
    EXPECT_FALSE(InteractiveCleanupSample::graspHorizontalAligned(target,gripper,0.0));
    target.z=0.542; gripper=target; gripper.y+=0.008;
    EXPECT_TRUE(InteractiveCleanupSample::graspHorizontalAligned(target,gripper,M_PI/2));
    gripper.y-=0.02;
    EXPECT_FALSE(InteractiveCleanupSample::graspHorizontalAligned(target,gripper,M_PI/2));
    gripper.x=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(InteractiveCleanupSample::graspHorizontalAligned(target,gripper,0.0));
  }
  static void recordedWrongPlaneWinner(bool measured)
  {
    auto n = make();
    n->has_locked_pointing_ = true;
    auto &p = n->locked_pointing_;
    // Capture 1792767718.609 from 20261023-225946-27894: the image
    // plane admits rabbit_doll although the arm points the other way.
    p.is_valid=true; p.plane_valid=true; p.pose_direction_valid=true;
    p.confidence=0.489951;
    p.origin.x=2.635726; p.origin.y=1.035299; p.origin.z=1.118583;
    p.direction.x=-0.895141; p.direction.y=-0.285141; p.direction.z=-0.342661;
    p.pose_direction=p.direction;
    p.plane_origin.x=-0.073543; p.plane_origin.y=-0.001267; p.plane_origin.z=1.013196;
    p.plane_normal.x=-0.329053; p.plane_normal.y=0.810833; p.plane_normal.z=0.484019;
    p.plane_forward.x=-0.143406; p.plane_forward.y=0.463710; p.plane_forward.z=-0.874304;
    p.camera_forward.x=0.949452; p.camera_forward.y=0.310384; p.camera_forward.z=0.046940;
    DetectedObject rabbit;
    rabbit.class_name="rabbit_doll"; rabbit.confidence=0.827;
    rabbit.has_3d_position=true;
    rabbit.position_3d.x=4.021; rabbit.position_3d.y=1.957; rabbit.position_3d.z=0.097;
    if (measured) {
      // A measured ray toward this same object remains authoritative even
      // if the separate pose estimate disagrees. No class-specific blacklist.
      p.depth_direction_valid=true;
      p.direction.x=rabbit.position_3d.x-p.origin.x;
      p.direction.y=rabbit.position_3d.y-p.origin.y;
      p.direction.z=rabbit.position_3d.z-p.origin.z;
      const double norm=std::hypot(std::hypot(p.direction.x,p.direction.y),p.direction.z);
      p.direction.x/=norm; p.direction.y/=norm; p.direction.z/=norm;
    }
    for (int i=0; i<6; ++i) {
      DetectedObjectArray frame;
      frame.objects.push_back(rabbit);
      auto soy=rabbit; soy.class_name="soysauce"; soy.confidence=0.946;
      soy.position_3d.x=1.845; soy.position_3d.y=0.849; soy.position_3d.z=0.054;
      frame.objects.push_back(soy); n->obs_objects_.push_back(frame);
    }
    EXPECT_EQ(n->selectStablePointedTarget(), measured);
    EXPECT_EQ(n->target_valid_, measured);
    if (measured) { EXPECT_EQ(n->target_class_, "rabbit_doll"); }
  }
  static void recordedPlaneAmbiguity(int variant)
  {
    auto n = make();
    n->has_locked_pointing_ = true;
    auto &p = n->locked_pointing_;
    p.is_valid = true; p.plane_valid = true;
    p.pose_direction_valid = true; p.confidence = 0.7;
    p.origin.x=2.776932; p.origin.y=1.141266; p.origin.z=0.936686;
    p.direction.x=-0.443543; p.direction.y=-0.133278; p.direction.z=-0.886288;
    p.plane_origin.x=-0.030718; p.plane_origin.y=0.011746; p.plane_origin.z=1.026865;
    p.plane_normal.x=-0.373751; p.plane_normal.y=0.927264; p.plane_normal.z=-0.022160;
    p.plane_forward.x=-0.019352; p.plane_forward.y=-0.031682; p.plane_forward.z=-0.999311;
    p.camera_forward.x=0.937353; p.camera_forward.y=0.312901; p.camera_forward.z=-0.153174;
    if (variant == 1) { p.direction.x *= -1; p.direction.y *= -1; }
    if (variant == 2) { p.origin.x=5; p.origin.y=2.5; } // Both candidates ahead.
    if (variant == 3) { p.direction.x=0; p.direction.y=0; }
    if (variant == 4) p.pose_direction_valid=false;
    if (variant == 5) p.confidence=0.3;
    for (int i=0; i<6; ++i) {
      DetectedObjectArray frame;
      DetectedObject a; a.class_name="soysauce"; a.confidence=0.94;
      a.has_3d_position=true; a.position_3d.x=1.843; a.position_3d.y=0.851; a.position_3d.z=0.054;
      auto b=a; b.class_name="rabbit_doll"; b.confidence=0.87;
      b.position_3d.x=4.017; b.position_3d.y=1.958; b.position_3d.z=0.093;
      frame.objects={a,b}; n->obs_objects_.push_back(frame);
    }
    EXPECT_EQ(n->selectStablePointedTarget(), variant == 0);
    if (variant == 0) { EXPECT_EQ(n->target_class_, "soysauce"); }
  }
  static std::shared_ptr<InteractiveCleanupSample> make(bool simulation_clock = false)
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", simulation_clock)});
    auto n = std::make_shared<InteractiveCleanupSample>(options);
    n->pub_head_trajectory_ = n->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/cleanup_timing_test/head", 10);
    n->pub_base_twist_ = n->create_publisher<geometry_msgs::msg::Twist>(
      "/cleanup_timing_test/base", 10);
    n->pub_vision_enable_ = n->create_publisher<std_msgs::msg::Bool>(
      "/cleanup_timing_test/vision", 10);
    n->reset();
    n->step_ = n->WaitForPickCommand;
    return n;
  }
  static PointingDirection gesture(int64_t ns)
  {
    PointingDirection p;
    p.header.stamp = rclcpp::Time(ns);
    p.received_steady_ns = InteractiveCleanupSample::ingressNowNs();
    p.is_valid = true; p.confidence = 0.9; p.person_is_valid = true;
    p.person_pixel_x = 320; p.person_pixel_y = 240;
    p.wrist_pixel_x = 300; p.wrist_pixel_y = 200;
    p.point_pixel_x = 380; p.point_pixel_y = 260;
    p.origin.x = 1; p.origin.z = 0.5;
    p.direction.x = 1/std::sqrt(1.0625);
    p.direction.z = -0.25/std::sqrt(1.0625);
    return p;
  }
  static void early(bool command_during_observation)
  {
    auto n = make();
    auto now = n->now().nanoseconds();
    for (int i = 0; i < 3; ++i)
      n->pointingCallback(std::make_shared<PointingDirection>(
        gesture(now-600000000LL+i*300000000LL)));
    n->runStateMachine();
    ASSERT_EQ(n->step_, n->ObserveTarget);
    ASSERT_TRUE(n->early_pick_observing_);
    if (command_during_observation) {
      n->handleIncomingMessage("Pick_it_up!", "", "test");
      ASSERT_TRUE(n->received_pick_command_);
    }
    n->runStateMachine(); // Existing one-shot head aim.
    n->state_enter_time_ = std::chrono::steady_clock::now()-std::chrono::seconds(2);
    n->head_motion_ready_time_ = {}; // Advance the monotonic motion gate too.
    n->runStateMachine(); // Settle.
    for (int i = 0; i < 6; ++i) {
      auto frame = std::make_shared<DetectedObjectArray>();
      frame->header.stamp = rclcpp::Time(n->head_observation_not_before_ns_+i+1);
      frame->received_steady_ns = n->head_observation_not_before_ns_+i+1;
      DetectedObject obj;
      obj.class_name = "bear_doll"; obj.confidence = 0.9;
      obj.has_3d_position = true;
      obj.position_3d.x = 1.8; obj.position_3d.z = 0.3;
      frame->objects.push_back(obj);
      n->objectsCallback(frame);
    }
    n->state_enter_time_ = std::chrono::steady_clock::now()-std::chrono::seconds(2);
    n->runStateMachine();
    ASSERT_TRUE(n->target_valid_);
    ASSERT_EQ(n->target_class_, "bear_doll");
    if (!command_during_observation) {
      ASSERT_EQ(n->step_, n->WaitForPickCommand);
      ASSERT_TRUE(n->preconfirmed_target_);
      n->handleIncomingMessage("Pick_it_up!", "", "test");
      n->runStateMachine();
    }
    EXPECT_EQ(n->step_, n->WaitForCleanCommand);
    EXPECT_FALSE(n->received_pick_command_);
    EXPECT_TRUE(n->destination_pointing_history_.empty());
  }
  static void drain(bool simulation_clock = false)
  {
    auto n = make(simulation_clock);
    if (simulation_clock) {
      ASSERT_EQ(n->now().nanoseconds(), 0);
    }
    n->handleIncomingMessage("Pick_it_up!", "", "test");
    ASSERT_GT(n->destination_window_.begin_ns, 0);
    n->step_ = n->WaitForCleanCommand;
    auto live = gesture(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
    live.plane_valid = true;
    n->pointingCallback(std::make_shared<PointingDirection>(live));
    ASSERT_EQ(n->destination_pointing_history_.size(), 1u);
    n->destination_pointing_history_.clear();
    n->handleIncomingMessage("Clean_up!", "", "test");
    auto p = gesture(n->destination_window_.end_ns);
    p.plane_valid = true;
    p.received_steady_ns = n->destination_window_.end_ns;
    // Result delivered after Clean, RGB entered at/before its cutoff.
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    ASSERT_EQ(n->destination_pointing_history_.size(), 1u);
    p.received_steady_ns = n->destination_window_.end_ns+1;
    // Source clock moves backwards, but must not admit a post-Clean frame.
    p.header.stamp = rclcpp::Time(int64_t(1000000000));
    n->pointingCallback(std::make_shared<PointingDirection>(p));
    EXPECT_EQ(n->destination_pointing_history_.size(), 1u);
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->WaitForCleanCommand); // Must not resolve empty/early.
    n->reset();
    EXPECT_FALSE(n->destination_window_.accepts(1));
    EXPECT_FALSE(n->preconfirmed_target_);
  }
  static void previewFailure()
  {
    auto n = make();
    n->step_ = n->ObserveTarget; n->sub_step_ = 2;
    n->early_pick_observing_ = true; n->early_pick_attempted_ = true;
    n->state_timer_initialized_ = true;
    n->state_enter_time_ = std::chrono::steady_clock::now()-std::chrono::seconds(10);
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->WaitForPickCommand);
    EXPECT_FALSE(n->target_valid_);
    EXPECT_FALSE(n->early_pick_observing_);
  }
  static void distractorRecovery()
  {
    auto n = make();
    n->step_ = n->ObserveTarget; n->sub_step_ = 2;
    n->has_locked_pointing_ = true;
    n->locked_pointing_ = gesture(n->now().nanoseconds());
    n->target_aim_pose_valid_ = true;
    n->target_view_recovery_count_ = 0;
    DetectedObjectArray frame;
    DetectedObject obj;
    obj.class_name = "hourglass"; obj.confidence = 0.9;
    obj.has_3d_position = true;
    obj.position_3d.x = 2; obj.position_3d.y = 1.58; obj.position_3d.z = 0.25;
    frame.objects.push_back(obj); n->obs_objects_.push_back(frame);
    EXPECT_FALSE(n->hasObservedNonPersonObject());
    // A plausible target must retain the directed view, even before six votes.
    n->obs_objects_[0].objects[0].position_3d.y = 0;
    EXPECT_TRUE(n->hasObservedNonPersonObject());
    n->obs_objects_[0].objects[0].position_3d.y = 1.58;
    n->state_timer_initialized_ = true;
    n->state_enter_time_ = std::chrono::steady_clock::now()-std::chrono::seconds(4);
    n->runStateMachine();
    EXPECT_EQ(n->sub_step_, 6);
    EXPECT_EQ(n->target_view_recovery_count_, 1);
    EXPECT_TRUE(n->has_locked_pointing_);
    EXPECT_FALSE(n->target_valid_);
  }
  static void normalOrExpired(bool expired)
  {
    auto n = make();
    if (expired) {
      n->preconfirmed_target_ = true;
      n->target_valid_ = true;
      n->target_class_ = "bear_doll";
      n->preconfirmed_target_time_ =
        std::chrono::steady_clock::now()-std::chrono::seconds(60);
    }
    n->handleIncomingMessage("Pick_it_up!", "", "test");
    n->runStateMachine();
    EXPECT_EQ(n->step_, n->ObserveTarget);
    EXPECT_FALSE(n->preconfirmed_target_);
    EXPECT_FALSE(n->target_valid_);
    EXPECT_FALSE(n->early_pick_observing_);
  }
};
TEST(CleanupTiming, CachedTargetSkipsPostCommandObservation) { CleanupTimingAccess::early(false); }
TEST(CleanupTiming, TransientHandDoesNotRestartSearch)
{ CleanupTimingAccess::transientHandDoesNotRestartSearch(); }
TEST(CleanupTiming, DescentDoesNotReuseMovingServoAbortTolerance)
{ CleanupTimingAccess::descentKeepsEstablishedClosure(); }
TEST(CleanupTiming, RejectedShortFallbackCannotLockButStableLongerGestureCan)
{ CleanupTimingAccess::shortFallbackLock(); }
TEST(CleanupTiming, HighQualitySingleFrameCannotBypassSettlingOrTemporalSupport)
{
  for (int variant=0; variant<6; ++variant) CleanupTimingAccess::briefMeasuredGesture(variant);
}
TEST(CleanupTiming, ElevatedGraspMustCompleteInsertion)
{ CleanupTimingAccess::elevatedInsertionTolerance(); }
TEST(CleanupTiming, CompletionRequiresModeratorAcknowledgement)
{ CleanupTimingAccess::completionWaitsForModerator(); }
TEST(CleanupTiming, TrackingAllowsSettledAcquisitionButEscapesImageEdge)
{ CleanupTimingAccess::trackingAcquisitionWindow(); }
TEST(CleanupTiming, RecordedPlaneTieRequiresIndependentFrontBackAgreement)
{
  for (int variant=0; variant<6; ++variant)
    CleanupTimingAccess::recordedPlaneAmbiguity(variant);
}
TEST(CleanupTiming, UniqueImagePlaneWinnerCannotContradictArmBearing)
{
  CleanupTimingAccess::recordedWrongPlaneWinner(false);
  CleanupTimingAccess::recordedWrongPlaneWinner(true);
}
TEST(CleanupTiming, LoweringMustPreserveHorizontalGraspAlignment)
{ CleanupTimingAccess::loweredHorizontalAlignment(); }
TEST(CleanupTiming, PickDuringPreviewIsNotLost) { CleanupTimingAccess::early(true); }
TEST(CleanupTiming, LatePreCleanInferenceIsAcceptedButNewFramesAreNot) { CleanupTimingAccess::drain(); }
TEST(CleanupTiming, SystemStampedImagesWithUnstartedSimulationClock) { CleanupTimingAccess::drain(true); }
TEST(CleanupTiming, FailedPreviewDoesNotGiveUpBeforeCommand) { CleanupTimingAccess::previewFailure(); }
TEST(CleanupTiming, UnrelatedObjectDoesNotBlockOneShotViewRecovery) { CleanupTimingAccess::distractorRecovery(); }
TEST(CleanupTiming, NormalCommandStillStartsOriginalObservation) { CleanupTimingAccess::normalOrExpired(false); }
TEST(CleanupTiming, ExpiredPreviewFallsBackToOriginalObservation) { CleanupTimingAccess::normalOrExpired(true); }
TEST(CleanupTiming, OneFrameOrMovingGestureDoesNotStartPreview)
{
  auto p = CleanupTimingAccess::gesture(1000000000);
  EXPECT_FALSE(interactive_cleanup::stablePickupPreview({p}));
  auto q = p; q.header.stamp = rclcpp::Time(int64_t(1300000000));
  auto r = p; r.header.stamp = rclcpp::Time(int64_t(1700000000));
  EXPECT_TRUE(interactive_cleanup::stablePickupPreview({p,q,r}));
  r.origin.x += 0.5;
  EXPECT_FALSE(interactive_cleanup::stablePickupPreview({p,q,r}));
}

TEST(CleanupTiming, RetainedFullArmSurvivesHeadAndCropButNotExpiryOrReset)
{
  for (int variant=0; variant<3; ++variant) CleanupTimingAccess::retainedArmPickup(variant);
}

TEST(CleanupTiming, RecordedDestinationDepthConsensusExcludesPoseTransitions)
{ CleanupTimingAccess::recordedDestinationWithTransitionFrames(); }

TEST(CleanupTiming, DelayedInferenceCannotPassCaptureTimeHeadBarrier)
{ CleanupTimingAccess::captureBarrier(); }
TEST(CleanupTiming, ObjectEvidenceCanReplaceWrongAimButNotResolveConflictingInstances)
{
  CleanupTimingAccess::revisableHypotheses(false);
  CleanupTimingAccess::revisableHypotheses(true);
}

TEST(CleanupTiming, HeadTrackingUsesActuatorRangeAndMotionDuration)
{
  // At the old software cap, a left-edge person/arm still needs another turn.
  const double requested = 1.2-std::atan2(62.815-320.,554.);
  const auto m=interactive_cleanup::planHeadMotion(1.2,0,requested,0,.3);
  EXPECT_GT(m.pan,1.2);
  EXPECT_LE(m.pan,interactive_cleanup::HSR_HEAD_PAN_MAX);
  EXPECT_GE(m.duration,std::abs(m.pan-1.2));
  const auto limit=interactive_cleanup::planHeadMotion(0,0,9,-9,.3);
  EXPECT_DOUBLE_EQ(limit.pan,1.745);
  EXPECT_DOUBLE_EQ(limit.tilt,-1.570);
  EXPECT_GE(limit.duration,1.745);
}

TEST(CleanupTiming, HandshakeWaitsForVisionAndReadinessSurvivesTaskReset)
{ CleanupTimingAccess::readinessHandshake(); }

TEST(CleanupTiming, FutureSourceStampCannotBypassActualHeadMotion)
{ CleanupTimingAccess::futureCaptureDuringMotion(); }

TEST(CleanupTiming, VisiblePersonDuringMotionSuppressesBlindScanUntilLost)
{ CleanupTimingAccess::visibleDuringMotionDoesNotRestartSearch(); }

TEST(CleanupTiming, PhaseMembershipUsesIngressDespiteSourceClockOffset)
{ CleanupTimingAccess::phaseIgnoresSourceClock(); }
