"""Pure navigation policy: no ROS, motion, or file mutations."""
import json
import math
from pathlib import Path

POLICY_PATH=Path(__file__).parent/'config/navigation_efficiency_v1.json'

class FeedbackPending(RuntimeError):
    pass

def load_policy(path=POLICY_PATH):
    p=json.loads(Path(path).read_text())
    if p.get('version')!=1 or p.get('name')!='navigation_efficiency_v1':
        raise ValueError('unknown_navigation_efficiency_policy')
    if p['final_precision']!=[.06,.08,.06] or p['final_acceptance']!=[.085,.11]:
        raise ValueError('final_precision_must_remain_verified')
    if p['base_feedback_max_age_s']!=.7 or p['task_timeout_s'] is not None:
        raise ValueError('base_feedback_or_task_policy_changed')
    def finite_tree(v):
        if isinstance(v,dict):return all(finite_tree(x) for x in v.values())
        if isinstance(v,list):return all(finite_tree(x) for x in v)
        return type(v) not in (int,float) or math.isfinite(v)
    if not finite_tree(p):raise ValueError('invalid_navigation_policy_number')
    if not (0<p['door_lateral_m']<=.06 and 0<p['ordinary_pass_radius_m']<=.15
            and 0<p['door_pass_along_m']<=.12 and 0<p['door_entry_yaw_rad']<=.15
            and 0<p['door_linear_mps']<=.20 and 0<p['door_angular_rps']<=.8):
        raise ValueError('door_or_pass_envelope_changed')
    if (not p['retry_backoff_s'] or any(type(x) not in (int,float) or x<=0 or x>10 for x in p['retry_backoff_s'])
            or p['joint_staleness_alone_cancels_navigation'] is not False):
        raise ValueError('invalid_navigation_recovery_policy')
    return p

def xy(p):
    return [float(p['x']),float(p['y'])] if isinstance(p,dict) else list(p[:2])

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

def build_landmarks(waypoints,goal,doors,p):
    """Preserve original coordinates; add each matched portal's center.

    Door association requires consecutive inside/outside endpoints of the same
    exported door. Unmatched waypoints keep the legacy precise stop behaviour;
    they must be explicitly marked ordinary before relaxing an unknown doorway.
    """
    points=[list(map(float,v)) for v in waypoints]
    rows=[dict(pose=v,kind='ordinary',original_index=i) for i,v in enumerate(points)]
    for row in rows:
        if any(math.dist(row['pose'][:2],xy(d[k]))<=.3 for d in doors for k in ('inside_pose','outside_pose')):
            row['kind']='unmatched_door'
    used=set();centers={}
    for i in range(len(points)-1):
        for d in doors:
            ends=(xy(d['inside_pose']),xy(d['outside_pose']))
            forward=math.dist(points[i][:2],ends[0])<=p['door_match_m'] and math.dist(points[i+1][:2],ends[1])<=p['door_match_m']
            reverse=math.dist(points[i][:2],ends[1])<=p['door_match_m'] and math.dist(points[i+1][:2],ends[0])<=p['door_match_m']
            if not (forward or reverse):continue
            if i in used or i+1 in used:continue
            heading=math.atan2(points[i+1][1]-points[i][1],points[i+1][0]-points[i][0])
            meta=dict(door_id=d['id'],heading=heading,center=xy(d['center']),width=float(d['width']))
            rows[i].update(kind='door_entry',door=meta)
            rows[i+1].update(kind='door_exit',door=meta)
            # Endpoint position unchanged; yaw now follows the crossing axis.
            rows[i]['pose']=[*points[i][:2],heading]
            rows[i+1]['pose']=[*points[i+1][:2],heading]
            centers[i]=dict(pose=[*xy(d['center']),heading],kind='door_center',door=meta,original_index=None)
            used.update((i,i+1));break
    out=[]
    for i,row in enumerate(rows):
        out.append(row)
        if i in centers:out.append(centers[i])
    out.append(dict(pose=list(map(float,goal)),kind='final',original_index=len(points)))
    return out

def pass_assessment(row,pose,p):
    target=row['pose'];distance=math.dist(pose[:2],target[:2])
    if row['kind']=='final':return dict(passed=False,distance_m=distance)
    if row['kind']=='unmatched_door':
        return dict(passed=distance<=.06 and abs(wrap(pose[2]-target[2]))<=.08,distance_m=distance,
            acceptance='unmatched_door_keeps_original_precision')
    if row['kind']=='ordinary':
        return dict(passed=distance<=p['ordinary_pass_radius_m'],distance_m=distance,
            acceptance='position_only_no_stop_or_yaw_requirement')
    angle=row['door']['heading'];dx=target[0]-pose[0];dy=target[1]-pose[1]
    along=math.cos(angle)*dx+math.sin(angle)*dy
    lateral=abs(-math.sin(angle)*dx+math.cos(angle)*dy)
    yaw=abs(wrap(pose[2]-angle))
    passed=(abs(along)<=p['door_pass_along_m'] and lateral<=p['door_lateral_m']
        and (row['kind']!='door_entry' or yaw<=p['door_entry_yaw_rad']))
    return dict(passed=passed,distance_m=distance,along_m=along,lateral_m=lateral,yaw_error_rad=yaw,
        acceptance='door_axis_position_and_entry_heading')

def caps_for_pose(profile,pose,segment_distance,doors,final_goal,p,*,holding=False):
    from motion_speed import navigation_caps
    caps=navigation_caps(profile,segment_distance,holding=holding);zone='ordinary'
    for d in doors:
        center=xy(d['center']);angle=float(d['center'].get('yaw',0.))
        dx=pose[0]-center[0];dy=pose[1]-center[1]
        along=abs(math.cos(angle)*dx+math.sin(angle)*dy)
        lateral=abs(-math.sin(angle)*dx+math.cos(angle)*dy)
        if along<=p['door_zone_half_length_m'] and lateral<=float(d['width'])/2+.25:
            caps=[min(caps[0],p['door_linear_mps']),min(caps[1],p['door_linear_mps']),min(caps[2],p['door_angular_rps'])];zone='door';break
    if math.dist(pose[:2],final_goal[:2])<=p['final_slow_distance_m']:
        caps=[min(caps[0],p['door_linear_mps']),min(caps[1],p['door_linear_mps']),min(caps[2],p['door_angular_rps'])];zone='final_approach'
    return caps,zone

def feedback_status(ages,joints,p,*,initial=False):
    """Current base feedback stays mandatory; old fixed-arm feedback is advisory."""
    for key in ('scan','tf'):
        age=ages.get(key)
        if age is None:raise FeedbackPending('missing_navigation_feedback:'+key)
        if type(age) not in (int,float) or not math.isfinite(age) or age < -p['future_skew_s']:
            raise ValueError('invalid_navigation_clock:'+key)
        if age>p['base_feedback_max_age_s']:raise FeedbackPending('stale_navigation_feedback:'+key)
    if len(joints)!=6 or not all(type(v) in (int,float) and math.isfinite(v) for v in joints):
        raise ValueError('invalid_navigation_joint_feedback')
    age=ages.get('joints')
    if type(age) not in (int,float) or not math.isfinite(age) or age < -p['future_skew_s']:
        raise ValueError('invalid_navigation_clock:joints')
    if initial and age>p['initial_joint_max_age_s']:raise FeedbackPending('stale_start_joint_feedback')
    return dict(joints_advisory_stale=age>p['joint_warning_age_s'],ages_s=ages)

class RetryState:
    def __init__(self,policy):self.policy=policy;self.count=0;self.anchor=None
    def failed(self,pose,*,front_obstacle_m=None):
        progress=0. if self.anchor is None else math.dist(self.anchor[:2],pose[:2])
        if self.anchor is None or progress>=self.policy['progress_radius_m']:
            self.count=0;self.anchor=list(pose)
        self.count+=1
        times=self.policy['retry_backoff_s'];delay=times[min(self.count-1,len(times)-1)]
        return dict(retry=self.count,wait_s=delay,progress_m=progress,task_retained=True,
            suspected_blocking=front_obstacle_m is not None and front_obstacle_m<.65,
            state='waiting_for_obstacle_or_replanning' if self.count>=3 else 'retry_unfinished_landmark')
