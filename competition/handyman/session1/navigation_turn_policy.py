"""Shared signed-turn policy. No ROS; all inputs are current map-frame geometry."""
import math
import numpy as np

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

def heading_goal(pose,goal,path,*,xy_tolerance=.06):
    if not math.isfinite(xy_tolerance) or xy_tolerance<=0:raise ValueError('invalid_xy_tolerance')
    if math.dist(pose[:2],goal[:2])<=xy_tolerance:return goal[2]
    if path:
        nearest=min(range(len(path)),key=lambda i:math.dist(pose[:2],path[i][:2]))
        for p in path[nearest:]:
            if math.dist(pose[:2],p[:2])>=.35:
                return math.atan2(p[1]-pose[1],p[0]-pose[0])
    return math.atan2(goal[1]-pose[1],goal[0]-pose[0])

class ArrivalHeading:
    """Enter final yaw only after XY arrival; use existing acceptance for drift.

    The small hysteresis uses the navigation's existing final acceptance, not
    a wider new arrival tolerance. A new goal always clears the arrival latch.
    """
    def __init__(self,xy_tolerance=.06,xy_acceptance=.085):
        if not all(math.isfinite(v) for v in (xy_tolerance,xy_acceptance)) or not 0<xy_tolerance<=xy_acceptance:
            raise ValueError('invalid_xy_arrival_limits')
        self.xy_tolerance=xy_tolerance;self.xy_acceptance=xy_acceptance
        self.goal=None;self.arrived=False
    def select(self,pose,goal,path,*,controller_rotating=False):
        previous=self.arrived
        changed_goal=self.goal!=list(goal)
        if changed_goal:self.goal=list(goal);self.arrived=False
        distance=math.dist(pose[:2],goal[:2])
        if self.arrived and distance>self.xy_acceptance:self.arrived=False
        elif not self.arrived and (distance<=self.xy_tolerance or controller_rotating and distance<=self.xy_acceptance):self.arrived=True
        yaw=goal[2] if self.arrived else heading_goal(pose,goal,path,xy_tolerance=self.xy_tolerance)
        return yaw,changed_goal or previous!=self.arrived

class TurnPolicy:
    """Latch the chosen arc until completed; never flip sides on noisy commands."""
    def __init__(self):self.remaining=None;self.previous=None;self.last_choice=None;self.choice_pose=None;self.choice_target=None
    def reset(self):
        self.remaining=None;self.previous=None;self.choice_pose=None;self.choice_target=None;self.last_choice=None
    def can_reuse(self,pose,target_yaw,w):
        return (self.choice_pose is not None and math.dist(pose[:2],self.choice_pose[:2])<.04
                and abs(wrap(pose[2]-self.choice_pose[2]))<.10
                and abs(wrap(target_yaw-self.choice_target))<.10 and self.last_choice['delta_rad']*w>=0)
    def step(self,clearance,pose,command,target_yaw):
        vx,vy,w=command
        if not all(math.isfinite(x) for x in (*pose,*command,target_yaw)):
            raise ValueError('invalid_turn_feedback')
        if self.remaining is not None:
            progress=wrap(pose[2]-self.previous[2])
            if abs(progress)>.6 or math.dist(pose[:2],self.previous[:2])>.08:
                raise ValueError('turn_feedback_jump')
            self.remaining-=progress;self.previous=list(pose)
            if abs(self.remaining)<.04:self.reset();return (0.,0.,0.)
            delta=math.copysign(min(abs(self.remaining),.25),self.remaining)
            if not clearance.turn_path(pose,delta)['clear']:raise ValueError('no_clear_carry_turn_arc')
            return (0.,0.,math.copysign(min(.8,max(.06,abs(self.remaining)*1.5)),self.remaining))
        if abs(w)<1e-6:return command
        # Keep clear local steering under Nav2 ownership. Do not replace a
        # small curve with a new turn merely because lookahead yaw has changed.
        if abs(wrap(target_yaw-pose[2]))<.15:
            c,s=math.cos(pose[2]),math.sin(pose[2])
            clear=all(not clearance.occupied([pose[0]+dt*(c*vx-s*vy),
                pose[1]+dt*(s*vx+c*vy),pose[2]+dt*w]) for dt in np.linspace(0,.5,6))
            if clear:return command
        if self.can_reuse(pose,target_yaw,w):plan=self.last_choice
        else:
            plan=clearance.choose_turn(pose,target_yaw);self.last_choice=plan
            self.choice_pose=list(pose);self.choice_target=target_yaw
        # The Nav2 desired heading can equal current heading during small curve
        # corrections. Still check the actual angular command's swept footprint.
        if abs(plan['delta_rad'])<.04:
            if not clearance.turn_path(pose,w*.5)['clear']:raise ValueError('no_clear_carry_turn_arc')
        elif plan['delta_rad']*w<0 or math.hypot(vx,vy)<.01:
            if self.choice_pose!=list(pose):
                plan=clearance.choose_turn(pose,target_yaw);self.last_choice=plan
                self.choice_pose=list(pose);self.choice_target=target_yaw
            # Own pure rotations even when Nav2 chose the same side. Otherwise
            # DWB's slow RotateToGoal command can hit its XY-only progress timer
            # while yaw is still converging. Keep the existing 0.8rad/s cap.
            self.remaining=plan['delta_rad'];self.previous=list(pose)
            return (0.,0.,math.copysign(min(.8,max(.06,abs(self.remaining)*1.5)),self.remaining))
        # Check curved motion too, not merely a rotation at its starting XY.
        c,s=math.cos(pose[2]),math.sin(pose[2])
        for dt in np.linspace(0,.5,6):
            p=[pose[0]+dt*(c*vx-s*vy),pose[1]+dt*(s*vx+c*vy),pose[2]+dt*w]
            if clearance.occupied(p):raise ValueError('no_clear_carry_turn_arc')
        return command

def reset_navigation_goal(relay,cancel_all,ready):
    """Finish the old action before starting a fresh BT/path for another goal.

    Leave forwarding disabled if cancellation or feedback reacquisition fails.
    Never replay velocity produced for the previous path after the handoff.
    """
    relay.enabled=False
    relay.zero()
    cancel_all()
    pose=ready(stopped=True)
    relay.policy.reset()
    relay.path=[];relay.latest=None;relay.last_time=0.
    relay.waiting_dynamic=False
    relay.enabled=True
    return pose

def final_heading_command(pose,goal,yaw_tolerance=.04):
    """Once XY is accepted, never replay Nav2's competing XY correction turn."""
    error=wrap(goal[2]-pose[2])
    if abs(error)<=yaw_tolerance:return (0.,0.,0.)
    return (0.,0.,math.copysign(min(.8,max(.06,abs(error)*1.5)),error))

def final_pose_ready(pose,goal,xy_tolerance=.085,yaw_tolerance=.08):
    return math.dist(pose[:2],goal[:2])<=xy_tolerance and abs(wrap(pose[2]-goal[2]))<=yaw_tolerance
