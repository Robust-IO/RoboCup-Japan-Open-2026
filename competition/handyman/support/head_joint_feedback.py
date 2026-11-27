"""Head positioning needs its own joint feedback, not a synchronized RGBD set."""
import math,time
from canned_grasp_runtime import TOPICS
from outcome_transfer_policy import load

def face_source(runtime,goal=(0.,-.5)):
    goal=list(map(float,goal))
    if len(goal)!=2 or not all(map(math.isfinite,goal)) or not -.6<=goal[0]<=.6 or not -.8<=goal[1]<=0:raise ValueError('invalid_source_head_goal')
    def positions():
        m=runtime.received.get('joints')
        if m is None:raise ValueError('fresh_head_joint_feedback_required')
        stamp=runtime.stamp(m);age=(runtime.node.get_clock().now().nanoseconds-stamp)/1e9
        if not -.02<=age<=.6:raise ValueError('fresh_head_joint_feedback_required')
        values=dict(zip(m.name,m.position));p=[values[k] for k in ('head_pan_joint','head_tilt_joint')]
        if not all(map(math.isfinite,p)):raise ValueError('invalid_head_joint_feedback')
        return p,stamp
    runtime.state();current,stamp=positions();pub=None;sent=None;rows=[]
    policy=load()['feedback'];tolerance=policy['head_tolerance_rad']
    already=max(abs(x-y) for x,y in zip(current,goal))<=tolerance
    if already:
        rows=[dict(positions=current,joint_stamp_ns=stamp)]
        runtime.record(dict(event='head_pose_verified_from_own_joint_feedback',goal=goal,rows=rows,command_skipped_already_at_goal=True))
        return rows
    try:
        if not already:
            topic=TOPICS[4];pub=runtime.publisher(topic);runtime.ownership(topic)
            pub.publish(runtime.trajectory(['head_pan_joint','head_tilt_joint'],goal,2.));sent=time.monotonic()
            runtime.record(dict(event='task_head_goal_sent',goal=goal,duration_s=2.))
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            runtime.pump(.02)
            try:p,stamp=positions()
            except ValueError:continue
            if max(abs(x-y) for x,y in zip(p,goal))>tolerance or sent is not None and time.monotonic()-sent<2:
                rows=[];continue
            if rows and stamp-rows[-1]['joint_stamp_ns']<200_000_000:continue
            rows.append(dict(positions=p,joint_stamp_ns=stamp))
            if len(rows)>=policy['head_confirmation_frames']:
                runtime.record(dict(event='head_pose_verified_from_own_joint_feedback',goal=goal,rows=rows,command_skipped_already_at_goal=already))
                return rows
        raise ValueError('head_goal_not_confirmed_from_joint_feedback')
    finally:
        if pub is not None:runtime.node.destroy_publisher(pub)

class TaskHeadFeedback:
    def face_source(self,goal=(0.,-.5)):return face_source(self,goal)
