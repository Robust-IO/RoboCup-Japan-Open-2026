"""Try measured grasp at current pose, then normal/obstacle-offset furniture poses.

Virtual geometry is only a candidate filter. Every arrival is re-observed and
re-planned from new RGBD before a grasp is permitted.
"""
import json,math,uuid
from dataclasses import replace
import numpy as np
from grasp_runtime.autograsp.planner import Scene,plan
from grasp_runtime.autograsp.kinematics import forward
from grasp_runtime.autograsp.templates import select_template
from grasp_staging import candidates,NAVIGATION_GAPS_M
from cabinet_grasp import is_cabinet,front_aligned,front_heading

def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])

def transform_scene(scene,source_pose,dest_pose):
    """Horizontal base changes preserve z and transform the measured support plane."""
    R=rotation(source_pose[2]-dest_pose[2])
    t=rotation(-dest_pose[2])@np.r_[np.asarray(source_pose[:2])-dest_pose[:2],0.]
    def move(p):return np.asarray(p)@R.T+t
    normal=R@np.r_[-scene.plane[:2],1.]
    constant=-scene.plane[2]-normal@t
    plane=np.r_[-normal[:2]/normal[2],-constant/normal[2]]
    return Scene(move(scene.target),move(scene.environment),plane,scene.stamp_ns)

def approach_plan(scene,backend,target):
    policy,_=select_template(target,scene,backend.policy)
    policy=replace(policy,max_candidates=64,feasible_budget=8,preferred_strategy='side',
                   allowed_strategies=('side',),neutral_wrist_only=getattr(backend,'grasp_attempt_cycle',1)==1,
                   rotation_search=getattr(backend,'grasp_attempt_cycle',1)>1)
    decision=plan(scene,backend.hand,policy)
    if decision.get('selected') is None and getattr(backend,'grasp_attempt_cycle',1)>1:
        top=plan(scene,backend.hand,replace(policy,preferred_strategy='overhead',allowed_strategies=('overhead',)))
        if top.get('selected') is not None:return top
    return decision

def direct_plan(scene,backend,target):
    # "Direct" means keep this observation direction and use the normal grasp
    # executor's measured approach, not already be within 3.5 cm of hand reach.
    result=approach_plan(scene,backend,target)
    for candidate in result['candidates']:
        palm,_=forward(candidate['arm_goal'])
        candidate['base_alignment_distance_m']=float(np.linalg.norm(palm[:2]-np.asarray(candidate['palm_base_m'])[:2]))
    return result['candidates'][0] if result['candidates'] else None,result

def approach_translations(candidate):
    palm,_=forward(candidate['arm_goal'])
    delta=np.r_[np.asarray(candidate['palm_base_m'])[:2]-palm[:2],0.]
    # Match the executor: lateral finger alignment, then forward approach.
    points=[np.zeros(3)]
    if candidate.get('strategy')=='side' and abs(delta[1])>.004 and delta[0]<.15:
        points.append(np.array([delta[0]-.15,0.,0.]))
    points.append(np.array([points[-1][0],delta[1],0.]));points.append(delta)
    return points

def current_base_approach_clear(stages,candidate,start):
    selector=getattr(stages,'candidate_selector',None)
    if selector is None:return True
    points=approach_translations(candidate)
    for a,b in zip(points,points[1:]):
        for delta in np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/min(.025,selector.res/2))+1)):
            # A local correction is not a new furniture-navigation candidate.
            xy=np.asarray(start[:2])+(rotation(start[2])@delta)[:2]
            cell=selector.cell(xy)
            if not selector.valid(cell) or not selector.walk[cell]:return False
    return True

def current_arm_path_clear(backend,candidate):
    from grasp_runtime.autograsp.planner import _open_collision
    start=np.asarray(backend.rt.state()['joints'],float);goal=np.asarray(candidate['arm_goal'],float)
    raised=start.copy();raised[0]=min(.69,max(start[0],goal[0]+.08))
    orient=goal.copy();orient[0]=raised[0]
    def clear(joints,translation,contact=False):
        if abs(joints[2])>.01:return False
        joints=joints.copy();joints[2]=0.
        palm,R=forward(joints);palm+=translation
        return (_open_collision(backend.scene.environment,palm,R,backend.hand)<=3 and
                _open_collision(backend.scene.target,palm,R,backend.hand,allow_target_contact=contact)<=3)
    # Lift/extend at the observation point before approaching the target.
    for a,b in ((start,raised),(raised,orient)):
        for joints in np.linspace(a,b,max(2,int(np.max(abs(a[:5]-b[:5]))/.04)+1)):
            if not clear(joints,np.zeros(3)):return False
    points=approach_translations(candidate)
    for a,b in zip(points,points[1:]):
        for delta in np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.025)+1)):
            if not clear(orient,delta):return False
    for joints in np.linspace(orient,goal,max(2,int(abs(orient[0]-goal[0])/.02)+1)):
        if not clear(joints,points[-1],contact=np.max(abs(joints-goal))<.02):return False
    return True


def filter_current_approach(stages,decision,cancel):
    start=stages.state(cancel)['poses']['map'];backend=stages.grasp_backend
    accepted=[]
    if not front_aligned(backend.table,start):return dict(decision,candidates=[],selected=None,cabinet_front_required=True)
    for candidate in decision.get('candidates',[]):
        stages.check(cancel)
        if current_base_approach_clear(stages,candidate,start) and current_arm_path_clear(backend,candidate):accepted.append(candidate)
    return dict(decision,candidates=accepted,selected=accepted[0] if accepted else None,
                observation_direction_approach_checked=True)

def obstacle_distance(scene,pose,current_pose):
    if not len(scene.environment):return 10.
    transformed=transform_scene(scene,current_pose,pose)
    # Exclude the support surface: do not treat the intended table as side clutter.
    p=transformed.environment
    high=p[:,2]-(p[:,:2]@transformed.plane[:2]+transformed.plane[2])>.035
    p=p[high]
    return float(np.min(np.linalg.norm(p[:,:2],axis=1))) if len(p) else 10.

def prepare(stages,effect,cancel):
    backend=stages.grasp_backend;state=stages.state(cancel);start=state['poses']['map']
    backend.filter_approach_decision=lambda decision:filter_current_approach(stages,decision,cancel)
    scene=backend.scene
    direct,report=direct_plan(scene,backend,stages.target)
    backend.record(dict(event='current_observation_grasp_check',direct=direct is not None,
                        rejected=report['rejected'],candidate=direct))
    if direct is not None and front_aligned(backend.table,start):
        for candidate in report.get('candidates',[direct]):
            base_clear=current_base_approach_clear(stages,candidate,start)
            arm_clear=current_arm_path_clear(backend,candidate) if base_clear else None
            backend.record(dict(event='observation_pose_approach_check',candidate_id=candidate.get('id'),
                                base_path_clear=base_clear,arm_path_clear=arm_clear,
                                reason='base_approach_blocked' if not base_clear else
                                       ('arm_path_blocked' if not arm_clear else 'clear')))
            if base_clear and arm_clear:
                backend.record(dict(event='observation_pose_retained_for_grasp',candidate=candidate,
                                    navigation_to_new_candidate=False,measured_close_approach_allowed=True))
                backend.pending_grasp_scene=backend.scene
                return
    if backend.holding_state()!='empty':raise ValueError('grasp_staging_requires_empty_hand')
    target=np.median([stages.map_point(backend.source_snapshot,p) for p in scene.target],axis=0)
    polygon=stages.environment['rooms'][stages.source_room]['region']
    selector=stages.candidate_selector
    # Navigation ends at an observation/staging standoff. The grasp executor
    # owns raising/extending the arm and the subsequent measured base approach.
    # Reach filtering concerns the final approach to the target, not arm reach
    # while standing one metre outside the furniture.
    from perception_bridge import inside
    normal=candidates(backend.table,target,polygon,base_radius=selector.radius,
                      gaps=NAVIGATION_GAPS_M)
    if is_cabinet(backend.table):normal=[p for p in normal if front_aligned(backend.table,p,1e-5)]
    ranked,audit=selector.rank(start,normal,stages.source_id)
    audit.update(navigation_standoff_from_furniture_edge_m=list(NAVIGATION_GAPS_M),
                 distance_reference='base_center_to_furniture_edge',close_approach_owner='grasp_executor')
    (stages.output/'grasp-staging-selection.json').write_text(json.dumps(audit,indent=2))
    def options():
        deferred=[]
        # Lazy planning: never evaluate all opposite-side poses before trying
        # an already clear normal candidate. Batch map queries for side offsets.
        # Rejected normal poses are offset seeds, never navigation goals.
        # Each offset receives its own reachability and sightline check below.
        seeds=list(ranked)+[p for p in normal if p not in ranked]
        for pose in seeds:
            if pose in ranked:
                virtual=transform_scene(scene,start,pose)
                possible=approach_plan(virtual,backend,stages.target)['selected']
                if possible is not None:yield pose,0.,'normal'
                else:deferred.append((pose,0.,'normal_requires_fresh_observation'))
            if is_cabinet(backend.table):continue
            offsets=[]
            for angle in (15,10,5):
                distance=math.dist(pose[:2],target[:2])*math.tan(math.radians(angle))
                left=np.array([-math.sin(pose[2]),math.cos(pose[2])])
                for sign in (-1,1):
                    xy=np.asarray(pose[:2])+left*distance*sign
                    p=[*map(float,xy),pose[2]-sign*math.radians(angle)]
                    if inside(*p[:2],polygon):offsets.append((p,angle,sign))
            accepted,offset_audit=selector.rank(start,[p for p,_,_ in offsets],stages.source_id)
            backend.record(dict(event='grasp_offset_candidates_checked',normal_pose=pose,
                                normal_reachable=pose in ranked,accepted_count=len(accepted),audit=offset_audit))
            alternatives=[(p,a,sign,obstacle_distance(scene,p,start)) for p,a,sign in offsets if p in accepted]
            if alternatives:
                direction=max(alternatives,key=lambda x:(x[3],x[1]))[2]
                alternatives.sort(key=lambda x:(x[2]!=direction,-x[1]))
                for p,angle,_,_ in alternatives:
                    candidate=approach_plan(transform_scene(scene,start,p),backend,stages.target)['selected']
                    if candidate is not None:yield p,angle,'obstacle_offset'
                    else:deferred.append((p,angle,'offset_requires_fresh_observation'))
        # Old-view geometry ranks observation opportunities; it cannot veto all
        # reachable viewpoints before the robot gets a fresh view there.
        yield from deferred
    staging_round=uuid.uuid4().hex
    checked=[]
    for index,(goal,angle,basis) in enumerate(options()):
        stages.check(cancel)
        if any(math.dist(goal[:2],p[:2])<.02 for p in checked):continue
        checked.append(goal)
        state=stages.state(cancel)
        if abs(state['joints'][1])>.05:
            from withdrawal_observer import WithdrawalObserver
            from table_withdrawal import TableWithdrawal
            TableWithdrawal(stages.runtime,stages.motion,WithdrawalObserver(backend,backend.table,lambda:None),label='adaptive-grasp-withdraw')()
        stages.runtime.prepare_navigation_from_current()
        backend.record(dict(event='adaptive_grasp_staging',goal=goal,offset_degrees=angle,basis=basis,staging_round=staging_round,candidate_index=index))
        try:
            stages.navigate(dict(effect,operation_id=effect['operation_id']+f'-grasp-staging-{staging_round}-{index}'),cancel,goal,stages.source_room)
        except Exception as exc:
            if 'navigation_candidate_exhausted' not in str(exc):raise
            continue
        if is_cabinet(backend.table):
            state=stages.state(cancel);pose=state['poses']['map'];yaw=front_heading(backend.table)
            delta=math.atan2(math.sin(yaw-pose[2]),math.cos(yaw-pose[2]))
            if abs(delta)>.04:
                stages.motion.adjust('cabinet-front-heading',pose,[0.,0.,delta],state['joints'][:5],state['joints'][5])
            if not front_aligned(backend.table,stages.state(cancel)['poses']['map']):continue
        backend.identity_confirmed=False;backend.pose_locked=False
        stages.reacquire_after_staging(cancel)
        report=approach_plan(backend.scene,backend,stages.target)
        backend.record(dict(event='staging_arrival_grasp_check',approach_feasible=report['selected'] is not None,
                            close_approach_owner='grasp_executor',rejected=report['rejected']))
        if report['selected'] is not None:
            pose=stages.state(cancel)['poses']['map']
            for candidate in report['candidates']:
                if current_base_approach_clear(stages,candidate,pose) and current_arm_path_clear(backend,candidate):
                    backend.pending_grasp_scene=backend.scene
                    return
    from grasp_runtime.autograsp.coordinator import AttemptFailure
    from grasp_runtime.autograsp.recovery import Failure
    raise AttemptFailure(Failure.NO_CANDIDATE,'grasp_staging_views_exhausted_requires_new_observation')
