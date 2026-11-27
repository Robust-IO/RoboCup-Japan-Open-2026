"""One wiring point for all six sessions' navigation and finite base rotations."""
import json
import math
from task_recovery import retry_read
from pathlib import Path
from threading import Event
import numpy as np
from carry_navigation import CarryNavigation

class SharedTurnGuard:
    def __init__(self,stages):
        self.stages=stages;self.motion=stages.motion;self.busy=False;self.serial=0
        self.cancel=getattr(self.motion,'session_cancel_event',Event())
        self.turner=CarryNavigation(stages,strict_checks=False)
        self.original_adjust=self.motion.adjust;self.original_command=self.motion.command
        self.motion.adjust=self.adjust;self.motion.command=self.command
        self.motion.turn_guard=self
        self.map_path=stages.output/'turn-map.npz'
        s=stages.candidate_selector
        np.savez_compressed(self.map_path,free=s.free,resolution=s.res,origin=s.origin,radius=s.radius)
        if hasattr(stages.runtime,'base_move'):
            self.original_base_move=stages.runtime.base_move
            stages.runtime.base_move=self.base_move
    def geometry(self):
        return retry_read(self.turner.geometry,cancel=self.cancel,record=self.stages.grasp_backend.record)
    def envelope(self):
        g=self.geometry();self.serial+=1
        path=self.stages.output/f'turn-envelope-{self.serial}.json'
        path.write_text(json.dumps(dict(polygon=g.polygon.tolist(),turn_radius_m=g.radius,map_archive=str(self.map_path))))
        return path
    def turn(self,yaw):
        self.busy=True
        try:return self.turner.turn(self.geometry(),yaw,self.cancel)
        finally:self.busy=False
    def adjust(self,name,pose,delta,arm,opening,**kw):
        if self.busy or abs(delta[2])<1e-8:return self.original_adjust(name,pose,delta,arm,opening,**kw)
        # Preserve the requested XY endpoint, but separate rotation from translation
        # so the long alternate arc cannot sweep the held object while translating.
        if math.hypot(*delta[:2])>1e-8:
            self.original_adjust(name+'-translation',pose,[delta[0],delta[1],0.],arm,opening,**kw)
        self.turn(pose[2]+delta[2])
        path=self.stages.output/(name+'.json')
        path.write_text(json.dumps(dict(status='endpoint_verified',basis='measured_signed_turn',rotation_guarded=True)))
        return str(path)
    def base_move(self,delta,arm,label):
        if abs(delta[2])<1e-8:return self.original_base_move(delta,arm,label)
        rt=self.stages.runtime;rt.sample(profile='state');s=rt.state()
        self.adjust(label,s['poses']['map'],delta,arm,s['joints'][5])
        return rt.state()
    def archive_report(self,path,suffix):
        archived=path.with_name(path.stem+suffix+'.json')
        # Stop confirmation must follow the exact owned report after archival.
        gate=getattr(self.motion,'session_motion_gate',None)
        if gate is not None:gate.navigation_report_path=archived
        path.rename(archived)
        return archived
    @staticmethod
    def recovery_heading(args,report,pose):
        reached=[x.get('target') for x in report.get('reached',[]) if x.get('target')]
        points=[]
        for i,arg in enumerate(args):
            if arg=='--waypoint':
                p=list(map(float,args[i+1:i+4]))
                if not any(math.dist(p[:2],q[:2])<.01 for q in reached):points.append(p)
        i=args.index('--goal');points.append(list(map(float,args[i+1:i+4])))
        for p in points:
            if math.dist(p[:2],pose[:2])>.10:return math.atan2(p[1]-pose[1],p[0]-pose[0])
        return points[-1][2]
    def command(self,args,*a,**kw):
        args=list(args);name=Path(str(args[0])).name
        if name not in ('navigate_carry_trial.py','efficient_navigation.py','room_entry_navigation.py','carry_navigation_runtime.py'):
            return self.original_command(args,*a,**kw)
        # Inner session-specific wrappers may already replace the executable.
        # Always replace their old envelope with the current arm/held geometry.
        if '--carry-envelope' in args:
            i=args.index('--carry-envelope');del args[i:i+2]
        args[0]=Path(__file__).with_name('carry_navigation_runtime.py')
        attempts=0;blocked_poses=[]
        while True:
            self.stages.check(self.cancel)
            from carry_drop_recovery import enabled,checkpoint
            inspect_carry=enabled(self.stages)
            original_stage=getattr(self.stages.motion,"stage",None)
            current=[*args,'--carry-envelope',self.envelope()]
            if inspect_carry:current+=['--monitor-carry-grip']
            try:return self.original_command(current,*a,**kw)
            except Exception as exc:
                report_path=Path(str(current[current.index('--output')+1]))
                report=json.loads(report_path.read_text()) if report_path.exists() else {}
                self.stages.check(self.cancel)
                reason=str(report.get('error',''))
                if reason.startswith('navigation_candidate_exhausted') and report.get('nav_reset') and report.get('carry_footprint_restored'):
                    raise ValueError(reason) from exc
                if inspect_carry and reason in ('carry_inspection_checkpoint','turn_feedback_jump'):
                    if not report.get('nav_reset') or not report.get('carry_footprint_restored') or report.get('cancel_error'):raise
                    attempts+=1
                    self.archive_report(report_path,f'-carry-check-{attempts}')
                    checkpoint(self.stages,self.cancel)
                    # Remove only waypoints physically reached by this child.
                    reached=[x.get('target') for x in report.get('reached',[]) if x.get('target')]
                    rebuilt=[];i=0
                    while i<len(args):
                        if args[i]=='--waypoint':
                            point=list(map(float,args[i+1:i+4]))
                            if not any(np.linalg.norm(np.asarray(point[:2])-v[:2])<.01 for v in reached):rebuilt.extend(args[i:i+4])
                            i+=4
                        else:rebuilt.append(args[i]);i+=1
                    args=rebuilt
                    if '--holding-opening' in args:
                        args[args.index('--holding-opening')+1]=self.stages.context['holding_opening_rad']
                    self.stages.motion.stage=original_stage
                    continue
                recoverable=reason.startswith(('carry_replan_required:','carry_no_progress','navigation_goal_rejected','navigation_service_or_goal_response_timeout','navigation_lifecycle_failed','controller_parameter_service_unavailable','navigation_manager_unavailable'))
                if not recoverable:raise
                if not report.get('nav_reset') or not report.get('carry_footprint_restored'):raise
                # The action is cancelled and velocity publisher removed before
                # translating without yaw; retry the same original destination.
                attempts+=1
                self.archive_report(report_path,f'-blocked-turn-{attempts}')
                self.stages.grasp_backend.record(dict(event='navigation_recovery',attempt=attempts,reason=reason,task_retained=True))
                if not any(tag in reason for tag in ('turn_sweep_blocked','dynamic_sweep_blocked','stalled_landmark')):
                    if self.cancel.wait(min(.5*attempts,3.)):self.stages.check(self.cancel)
                    continue
                self.busy=True
                try:
                    self.stages.runtime.sample(profile='state')
                    current_pose=self.stages.runtime.state()['poses']['map']
                    blocked_poses.append(list(current_pose))
                    heading=self.recovery_heading(args,report,current_pose)
                    self.turner.escape(self.geometry(),self.cancel,excluded=blocked_poses[-8:],target_yaw=heading)
                except ValueError as recovery_error:
                    if str(recovery_error) not in ('no_translation_only_carry_escape_on_static_map','carry_escape_endpoint_not_clear','no_clear_carry_turn_arc'):raise
                    self.stages.grasp_backend.record(dict(event='navigation_escape_retry',
                        attempt=attempts,reason=str(recovery_error),next_action='fresh_geometry_and_navigation_replan',task_retained=True))
                    if self.cancel.wait(min(.5*attempts,2.)):self.stages.check(self.cancel)
                finally:
                    self.busy=False
                    self.stages.motion.stage=original_stage
    def prepare_sweep(self):
        rt=self.stages.runtime;rt.sample(profile='state');pose=rt.state()['poses']['map'];g=self.geometry()
        # A 360 degree search has no shorter alternate arc. Move to a location
        # where the entire sweep fits, preserving complete angular coverage.
        if not g.turn_path(pose,2*math.pi)['clear']:
            self.busy=True
            try:self.turner.escape(g,self.cancel)
            finally:self.busy=False
        return self.geometry()

def install(stages):
    if getattr(stages.motion,'turn_guard',None) is not None:return stages.motion.turn_guard
    guard=SharedTurnGuard(stages)
    return guard
