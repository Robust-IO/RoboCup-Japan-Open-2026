"""Session3 carry navigation: measured envelope, escape/replan, visual checkpoints."""
import json,math,uuid
from pathlib import Path
import numpy as np
from carry_clearance import CarryClearance
from hold_observer import HoldObserver

class CarryNavigation:
    def __init__(self,stages,*,strict_checks=True):self.stages=stages;self.serial=0;self.strict_checks=strict_checks;self.tag=uuid.uuid4().hex[:8]
    def geometry(self):
        b=self.stages.grasp_backend;item=b._sample();s=item['snapshot'];p=b.rt.palm_pose(s)
        from hand_camera_ray import rotation
        opening=float(s['joints']['hand_motor_joint']);parts=b.physical.sample(opening)['parts']
        points=[np.asarray(x['vertices_palm_m']) for x in parts if not x.get('is_trigger')]
        if self.stages.context is not None and opening<.6:points.append(np.asarray(self.stages.context['vertices_palm_m']))
        base=np.concatenate(points)@rotation(p['quaternion']).T+p['translation']
        if getattr(b,'self_geometry',None) is not None and s.get('base_from_self_links_at_depth'):
            base=np.concatenate([base,*b.self_geometry.boxes(s['base_from_self_links_at_depth'])])
        b.rt.fresh_measurement(s)
        return CarryClearance(self.stages.candidate_selector,base)
    def check_hold(self,label):
        try:
            proof=HoldObserver(self.stages.grasp_backend,self.stages.context)(label)
            if proof.get('holding_verified') is not True or proof.get('visual_confirmation') is not True:raise ValueError('visual_hold_required')
            return proof
        except ValueError as exc:
            state=self.stages.runtime.state()
            report=dict(status='recovery_required',reason=str(exc),label=label,target=self.stages.target,
                last_robot_pose=state['poses']['map'],object_location_known=False,
                task_finished_allowed=False,automatic_floor_regrasp_available=False)
            (self.stages.output/'carry-recovery-required.json').write_text(json.dumps(report,indent=2))
            raise ValueError('held_object_reobservation_required:'+str(exc)) from exc
    def escape(self,clearance,cancel,excluded=(),*,target_yaw=None):
        self.stages.check(cancel);rt=self.stages.runtime;rt.sample(profile='state');state=rt.state()
        plan=clearance.escape(state['poses']['map'],excluded=excluded,**({'target_yaw':target_yaw} if target_yaw is not None else {}))
        if plan['distance_m']:
            self.serial+=1;self.stages.motion.stage='withdraw'
            self.stages.motion.adjust('carry-turn-space-'+self.tag+'-'+str(self.serial),state['poses']['map'],plan['delta'],state['joints'][:5],state['joints'][5])
            self.stages.check(cancel);rt.sample(profile='state')
            actual=rt.state()['poses']['map']
            if target_yaw is None:
                if not clearance.turn_clear(actual):raise ValueError('carry_escape_endpoint_not_clear')
            else:clearance.choose_turn(actual,target_yaw)
            if self.strict_checks:self.check_hold('after-carry-retreat')
        return plan
    def turn(self,clearance,target_yaw,cancel,*,allow_escape=True):
        """Keep the selected sign: a single wrapped Nav2 goal cannot do that."""
        stages=self.stages;rt=stages.runtime
        stages.check(cancel);rt.sample(profile='state');state=rt.state()
        try:plan=clearance.choose_turn(state['poses']['map'],target_yaw)
        except ValueError as exc:
            if str(exc)!='no_clear_carry_turn_arc' or not allow_escape:raise
            self.escape(clearance,cancel,target_yaw=target_yaw)
            rt.sample(profile='state');state=rt.state()
            plan=clearance.choose_turn(state['poses']['map'],target_yaw)
        self.serial+=1
        audit=stages.output/f'carry-turn-choice-{self.tag}-{self.serial}.json'
        record=dict(start=state['poses']['map'],target_yaw=target_yaw,plan=plan,steps=[])
        audit.write_text(json.dumps(record,indent=2))
        remaining=plan['delta_rad']
        while abs(remaining)>.04:
            stages.check(cancel);rt.sample(profile='state');state=rt.state();pose=state['poses']['map']
            delta=math.copysign(min(.30,abs(remaining)),remaining)
            if not clearance.turn_path(pose,delta)['clear']:raise ValueError('carry_turn_arc_changed')
            self.serial+=1;stages.motion.stage='withdraw'
            adjust=getattr(getattr(stages.motion,'turn_guard',None),'original_adjust',stages.motion.adjust)
            adjust(f'carry-directed-turn-{self.tag}-{self.serial}',pose,[0.,0.,delta],state['joints'][:5],state['joints'][5])
            stages.check(cancel);rt.sample(profile='state');actual=rt.state()['poses']['map']
            progress=math.atan2(math.sin(actual[2]-pose[2]),math.cos(actual[2]-pose[2]))
            record['steps'].append(dict(requested=delta,actual=progress,pose=actual))
            audit.write_text(json.dumps(record,indent=2))
            if math.dist(pose[:2],actual[:2])>.04 or progress*delta<=0 or abs(progress)<.005 or abs(progress-delta)>.10:
                raise ValueError('carry_directed_turn_feedback_mismatch')
            remaining-=progress
        rt.sample(profile='state');actual=rt.state()['poses']['map']
        if abs(math.atan2(math.sin(target_yaw-actual[2]),math.cos(target_yaw-actual[2])))>.08:
            raise ValueError('carry_directed_turn_endpoint_mismatch')
        record['completed']=True;audit.write_text(json.dumps(record,indent=2))
        return plan
    def run(self,goal,room,cancel,*,require_final_yaw=True):
        stages=self.stages;motion=stages.motion;rt=stages.runtime
        stages.context['require_visual_carry_checks']=False
        self.check_hold('before-handover-navigation')
        clearance=self.geometry()
        path=stages.output/'carry-envelope.json';path.write_text(json.dumps(dict(polygon=clearance.polygon.tolist(),turn_radius_m=clearance.radius)))
        rt.sample(profile='state');start=rt.state()['poses']['map'];source=motion.room_at(start)
        # Doorway entry/exit are existing validated waypoints, not invented turns.
        waypoints=[*motion.route(source,room),goal];previous_command=motion.command
        def command(args,*a,**kw):
            if Path(str(args[0])).name=='navigate_carry_trial.py':
                args=[Path(__file__).with_name('carry_navigation_runtime.py'),*args[1:],'--carry-envelope',path]
            return previous_command(args,*a,**kw)
        motion.command=command
        try:
            for index,point in enumerate(waypoints):
                excluded=[]
                while True:
                    stages.check(cancel);rt.sample(profile='state');state=rt.state();pose=state['poses']['map']
                    bearing=math.atan2(point[1]-pose[1],point[0]-pose[0])
                    if math.dist(point[:2],pose[:2])>.08:
                        self.turn(clearance,bearing,cancel)
                    rt.sample(profile='state');state=rt.state();pose=state['poses']['map']
                    self.serial+=1;name=f'moderator-carry-{index}-{self.serial}'
                    motion.stage='navigate_destination'
                    try:
                        # Own the final rotation as well, instead of allowing
                        # Nav2 to choose the shorter but obstructed side.
                        travel_goal=[point[0],point[1],bearing]
                        from contextlib import nullcontext
                        from room_entry import finish_on_room_entry
                        arrival=(finish_on_room_entry(motion,stages.output,room,stages.environment['rooms'][room]['region'],position_goal=point)
                                 if not require_final_yaw and index==len(waypoints)-1 else nullcontext())
                        with arrival:
                            result=motion.navigate(motion.room_at(pose),room,travel_goal,state['joints'][5],name,remaining_waypoints=[])
                    except Exception:
                        report_path=motion.output/(name+'.json')
                        report=json.loads(report_path.read_text()) if report_path.exists() else {}
                        error=report.get('error','')
                        if not (error.startswith('carry_no_progress') or error.startswith('carry_replan_required')):raise
                        if report.get('carry_footprint_restored') is not True or report.get('nav_reset') is not True:raise
                        # Owned goal has been cancelled and footprint restored before retreat.
                        self.check_hold('after-carry-navigation-stop')
                        rt.sample(profile='state');excluded.append(rt.state()['poses']['map'])
                        self.escape(clearance,cancel,excluded=excluded)
                        continue
                    report=json.loads((motion.output/(name+'.json')).read_text())
                    if not result.get('verified') or report.get('carry_footprint_restored') is not True:raise ValueError('carry_navigation_incomplete')
                    if require_final_yaw or index<len(waypoints)-1:
                        self.turn(clearance,point[2],cancel,allow_escape=False)
                    self.check_hold('after-carry-waypoint-'+str(index));break
        finally:motion.command=previous_command
        return dict(verified=True,envelope=str(path),visual_checkpoints=len(waypoints))
