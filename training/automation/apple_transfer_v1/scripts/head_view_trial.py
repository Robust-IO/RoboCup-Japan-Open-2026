"""Bounded single head-only trial, dry-run by default. No base/arm/task commands.

One finite 2-second JointTrajectory. Timeout is NOT trajectory cancellation.
RGB captures are not object detections, depth validation or absence evidence.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time

JOINTS=('head_pan_joint','head_tilt_joint')
COMMAND='/hsrb/head_trajectory_controller/command'
# Live integrated trace r7: JointState timestamps up to 14.105 ms ahead.
# Unity Header uses millisecond wall time minus a separately synchronized gap.
# Fixed 20 ms bound; do not learn arbitrary offsets or rewrite sensor stamps.
# Freshness (300 ms), ordering, and both stationary windows remain unchanged.
MAX_FUTURE_SKEW_NS=20_000_000
SEARCH_FEEDBACK_MAX_S=.5


def target(pan,tilt):
    if not all(math.isfinite(v) for v in (pan,tilt)) or not (-.6<=pan<=.6 and -.8<=tilt<=0):
        raise ValueError('outside_conservative_trial_range')
    return (pan,tilt)


class Settling:
    def __init__(self,goal,max_feedback_s=.3):
        if not math.isfinite(max_feedback_s) or not 0<max_feedback_s<=.5:
            raise ValueError('invalid_feedback_limit')
        self.max_feedback_s=max_feedback_s
        self.goal=target(*goal)
        self.reset()

    def reset(self):
        self.since=None;self.anchor=None;self.last_stamp=0;self.last_now=None
        self.reason='waiting_for_feedback'

    def sample(self,names,positions,stamp,sensor_now,now):
        try:
            if len(names)!=len(positions) or any(names.count(k)!=1 for k in JOINTS):
                raise ValueError('invalid_joint_names')
            pose=tuple(positions[names.index(k)] for k in JOINTS)
            if (not all(math.isfinite(v) for v in (*pose,now)) or
                type(stamp) is not int or type(sensor_now) is not int or
                stamp<=self.last_stamp or not -MAX_FUTURE_SKEW_NS<=sensor_now-stamp<=int(self.max_feedback_s*1e9)):
                raise ValueError('stale_or_invalid_feedback')
            gap=self.last_now is not None and (not 0<now-self.last_now<=self.max_feedback_s or stamp-self.last_stamp>int(self.max_feedback_s*1e9))
            self.last_stamp=stamp;self.last_now=now
            if gap or any(abs(v-g)>.03 for v,g in zip(pose,self.goal)):
                self.reason='feedback_gap' if gap else 'outside_target_tolerance'
                self.since=None;self.anchor=None
                return False,pose
            if self.anchor is None or any(abs(v-a)>.01 for v,a in zip(pose,self.anchor)):
                self.anchor=pose;self.since=(now,stamp)
            settled=now-self.since[0]>=.6 and stamp-self.since[1]>=600_000_000
            self.reason='settled' if settled else 'collecting_stationary_window'
            return settled,pose
        except (ValueError,TypeError,IndexError) as exc:
            self.reason=str(exc) if isinstance(exc,ValueError) else 'malformed_feedback'
            self.since=None;self.anchor=None
            return False,None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pan',type=float,default=0.)
    p.add_argument('--tilt',type=float,default=-.5)
    p.add_argument('--run',action='store_true')
    p.add_argument('--confirm-head-motion',action='store_true')
    p.add_argument('--output',type=Path)
    p.add_argument('--synthetic-test',action='store_true',help='Isolated domain 73 only; never live domain')
    a=p.parse_args();goal=target(a.pan,a.tilt)
    if not a.run:
        print(json.dumps(dict(started=False,head_only=True,topic=COMMAND,joints=JOINTS,
                              positions=goal,duration_s=2.,requires='fresh joint feedback and explicit confirmation')))
        return
    if not a.confirm_head_motion or a.output is None:
        p.error('run requires --confirm-head-motion and --output')
    domain='73' if a.synthetic_test else '71'
    if os.environ.get('ROS_DOMAIN_ID')!=domain or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('requires domain '+domain+' / localhost wrapper')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState,Image
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    from capture_rgb_readonly import encode_png
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Node('handyman_single_head_view_trial')
    pub=node.create_publisher(JointTrajectory,COMMAND,10)
    gate=Settling(goal);latest=None;sent=None;settled_stamp=None;frames=0;last_image=0
    records=[];failure=None;started=time.monotonic()
    feedback=[];image_rejections={};previous_receive=None

    def joint(msg):
        nonlocal latest,settled_stamp,previous_receive
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        now=time.monotonic()
        sensor_now=node.get_clock().now().nanoseconds
        ok,pose=gate.sample(list(msg.name),list(msg.position),stamp,sensor_now,now)
        if len(feedback)<1000:
            feedback.append(dict(elapsed_s=now-started,stamp_ns=stamp,age_s=(sensor_now-stamp)/1e9,
                receive_gap_s=None if previous_receive is None else now-previous_receive,
                positions=pose,raw_names=list(msg.name),
                raw_positions=[v if math.isfinite(v) else None for v in msg.position],
                reason=gate.reason,settled=ok,names_count=len(msg.name),positions_count=len(msg.position)))
        previous_receive=now
        if pose is None:
            latest=None;settled_stamp=None;return
        latest=(now,stamp,pose)
        if sent is not None and ok and now>=sent+2.:
            if settled_stamp is None:
                settled_stamp=stamp
                records.append(dict(event='head_settled',stamp_ns=stamp,positions=pose))
        else:settled_stamp=None

    def rgb(msg):
        nonlocal frames,last_image
        now=time.monotonic();stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        reason=None
        if settled_stamp is None:reason='head_not_settled'
        elif latest is None or now-latest[0]>.3:reason='head_feedback_not_fresh'
        elif stamp<=max(settled_stamp,last_image):reason='old_or_duplicate_image'
        elif not 0<=node.get_clock().now().nanoseconds-stamp<=300_000_000:reason='image_not_fresh'
        if reason:
            image_rejections[reason]=image_rejections.get(reason,0)+1
            return
        if frames>=3:return
        name=f'view-{frames:02d}.png'
        (a.output/name).write_bytes(encode_png(msg))
        records.append(dict(event='rgb_captured',file=name,stamp_ns=stamp,head_stamp_ns=latest[1],
                            head_positions=latest[2],detection_verified=False))
        last_image=stamp;frames+=1

    node.create_subscription(JointState,'/hsrb/joint_states',joint,qos_profile_sensor_data)
    node.create_subscription(Image,'/hsrb/head_rgbd_sensor/rgb/image_raw',rgb,qos_profile_sensor_data)
    try:
        while rclpy.ok() and time.monotonic()-started<12 and frames<3:
            rclpy.spin_once(node,timeout_sec=.03)
            now=time.monotonic()
            if sent is None:
                if now-started>4:raise RuntimeError('head_preflight_timeout_no_command_sent')
                if latest is None or now-latest[0]>.3 or pub.get_subscription_count()!=1:continue
                if not (-.65<=latest[2][0]<=.65 and -.85<=latest[2][1]<=.05):
                    raise RuntimeError('initial_head_pose_outside_trial_range')
                if max(abs(x-y) for x,y in zip(latest[2],goal))>1.:
                    raise RuntimeError('head_motion_too_large')
                msg=JointTrajectory();msg.joint_names=list(JOINTS)
                point=JointTrajectoryPoint();point.positions=list(goal);point.time_from_start.sec=2
                msg.points=[point];pub.publish(msg);sent=now;gate.reset();settled_stamp=None
                records.append(dict(event='head_command_sent',positions=goal,previous=latest[2],duration_s=2.))
        if frames!=3:failure='head_or_rgb_verification_timeout'
    except (KeyboardInterrupt,RuntimeError,ValueError) as exc:
        failure=str(exc) or 'interrupted'
    finally:
        report=dict(head_only=True,command_sent=sent is not None,frames=frames,passed=frames==3 and failure is None,
                    failure=failure,events=records,does_not_exist_authorized=False,
                    feedback_samples=len(feedback),last_feedback_reason=gate.reason,image_rejections=image_rejections,
                    caveat='One finite trajectory; timeout is not trajectory cancellation. No automatic return motion.')
        (a.output/'feedback.json').write_text(json.dumps(feedback,indent=2,allow_nan=False))
        (a.output/'summary.json').write_text(json.dumps(report,indent=2))
        node.destroy_node();rclpy.try_shutdown()
        print(json.dumps(report))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
