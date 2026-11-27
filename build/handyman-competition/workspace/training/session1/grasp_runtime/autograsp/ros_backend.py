"""ROS adapter for measured side/overhead grasps; no per-class dispatch.

Requires RobotLock held by the caller. Only the gripper geometry is checked here;
whole-arm motion planning is not claimed. Occluded RGBD triggers withdrawal and
new observation, never a fabricated target measurement or manual pixel anchor.
"""
import math
import numpy as np
from .coordinator import AttemptFailure, LiftEvidence
from .recovery import Failure
from .planner import Scene, evaluate, Policy
from .kinematics import CONTACT, forward
from .tracking import register_surface, outside_boxes, dominant_component, UnobservedSurface
from .contact import contact_fit
from .grasp_confirmation import grip_retained, lift_sample_passes


def runtime_class():
    from horizontal_can_trial import HorizontalCanRuntime
    from stopped_capture_runtime import with_stopped_capture
    from outcome_transfer_runtime import OutcomeRuntime
    from hand_camera_ray import rotation
    class GenericRuntime(HorizontalCanRuntime):
        # Use the current outcome-based closure feedback rather than the old
        # adapter's fixed-centre can closure policy.
        close_gripper=OutcomeRuntime.close_gripper
        def face_source(self,goal=(0.,-.5)):
            import time
            from canned_grasp_runtime import TOPICS
            goal=np.asarray(goal,float)
            if goal.shape!=(2,) or not np.isfinite(goal).all() or not -.6<=goal[0]<=.6 or not -.8<=goal[1]<=0:
                raise ValueError('invalid_source_head_goal')
            self.sample(profile='state')
            if np.max(abs(np.asarray(self.observation_state()['head'])-goal))<=.02:
                self.record(dict(event='generic_head_already_at_goal',goal=goal.tolist()))
                return
            topic=TOPICS[4];pub=self.publisher(topic)
            try:
                self.ownership(topic)
                pub.publish(self.trajectory(['head_pan_joint','head_tilt_joint'],goal.tolist(),2.))
                self.record(dict(event='generic_head_command_sent',goal=goal.tolist()))
                sent=time.monotonic();stable=None
                while True:
                    self.pump(.02);self.ownership(topic)
                    state=self.observation_state()
                    if np.max(abs(np.asarray(state['head'])-goal))<=.02:
                        stable=time.monotonic() if stable is None else stable
                        if time.monotonic()-sent>=2. and time.monotonic()-stable>=.3:
                            self.record(dict(event='generic_head_feedback_confirmed',head=state['head']))
                            return
                    else:stable=None
            finally:self.node.destroy_publisher(pub)
        def contact_error(self,center,snapshot):
            palm=self.palm_pose(snapshot)
            return np.asarray(center)-np.asarray(palm['translation'])-rotation(palm['quaternion'])@CONTACT
        def is_aligned(self,center,snapshot):
            check=getattr(self,'actual_contact_check',None)
            if check is not None:return check(snapshot)['fits']
            error=self.contact_error(center,snapshot)
            return bool((abs(error)<=np.array([.009,.006,.006])).all())
    class SessionRuntime(with_stopped_capture(GenericRuntime)):
        def save(self,sample,name,measurement=None):
            from .capture_names import unused_capture_name
            return super().save(sample,unused_capture_name(self.root,name),measurement)
        def sample(self,*args,**kwargs):
            # Reject the mismatched capture and acquire a new one; never accept
            # future-dated evidence or weaken RGB/depth synchronization.
            import time
            started=time.monotonic()
            while True:
                try:return super().sample(*args,**kwargs)
                except ValueError as exc:
                    if str(exc)!='action_measurement_unsynchronised_or_clock_changed' or time.monotonic()-started>8:
                        raise
                    self.record(dict(event='resample_timestamp_mismatch',reason=str(exc),robot_commands_sent=False))
                    self.pump(.03)
    return SessionRuntime


class RosBackend:
    def __init__(self,runtime,worker,table,hand,physical_hand,record,policy=Policy()):
        self.rt=runtime; self.worker=worker; self.table=table
        self.hand=hand; self.physical=physical_hand; self.record=record; self.policy=policy
        self.serial=0; self.last_stamp=0; self.holding='uncertain'
        self.closed=False; self.baseline=None; self.lift_done=False
        self.evidence=None; self.candidate=None; self.retreat_steps=[]
        self.pose_locked=False
        self.rt.actual_contact_check=self._actual_contact_check
        self.self_geometry=None
        if hasattr(runtime,'buf'):
            from .self_geometry import ForearmGeometry
            self.self_geometry=ForearmGeometry('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/geometry/hsr-full-geometry-20261010-203028.json')

    def _current_plane(self,snapshot):
        R,t=self._tf(snapshot);R0,t0=self._tf(self.source_snapshot)
        n0=np.r_[-self.plane[:2],1.];nw=R0@n0;n=R.T@nw
        return np.r_[-n[:2]/n[2],(self.plane[2]+nw@t0-nw@t)/n[2]]

    def _actual_contact_check(self,snapshot):
        from hand_camera_ray import rotation
        fresh=(hasattr(self,'observed_contact_points') and
               snapshot['streams']['depth']['stamp_ns']==self.observed_contact_stamp)
        if fresh:
            palm=self.rt.palm_pose(snapshot)
            fit=contact_fit(self.observed_contact_points,palm['translation'],rotation(palm['quaternion']),
                            self._current_plane(snapshot),self.hand,self.policy,
                            require_pad_overlap=not getattr(self,"skip_actual_pad_overlap",False))
            fit.update(contact_basis='current_observed_surface',current_target_depth_measured=True)
            if fit.get('reason')=='contact_band_unobserved' and getattr(self,'skip_contact_band_depth',False):
                from .occluded_trial_close import session6_depth_override
                from .table_clearance import hand_table_clearance
                measurement=getattr(self,'last_tracking_measurement',None)
                if measurement:
                    return session6_depth_override(fit,measurement,snapshot['streams']['depth']['stamp_ns'],
                        self.rt.contact_error(measurement['center_m'],snapshot),self.candidate,self.policy,
                        hand_table_clearance(palm['translation'],rotation(palm['quaternion']),
                                            self._current_plane(snapshot),self.hand))
            if fit.get('reason')=='contact_band_unobserved':
                measurement=getattr(self,'last_tracking_measurement',None)
                stamp=snapshot['streams']['depth']['stamp_ns']
                if measurement and measurement.get('stamp_ns')==stamp:
                    from .occluded_trial_close import decide
                    R,t=self._tf(snapshot)
                    reference=contact_fit((self.template-t)@R,palm['translation'],rotation(palm['quaternion']),
                                          self._current_plane(snapshot),self.hand,self.policy)
                    fit=decide(fit,reference,measurement,stamp,
                               self.rt.contact_error(measurement['center_m'],snapshot))
            return fit
        return dict(fits=False,reason='actual_contact_requires_current_observed_points')

    def holding_state(self):
        value=float(self.rt.state()['joints'][5])
        if value>=.60:return 'empty'
        # A fully closed joint alone cannot exclude a very thin held object.
        return self.holding

    def restore_selected_plan(self,root,target):
        """Explicitly resume a previously selected candidate, without redetection.

        The caller authorizes use of this historical pose. Scene geometry is
        reconstructed from its actual saved RGBD, never invented depth.
        """
        import json,cv2
        from pathlib import Path
        from can_destination_surface import measure
        from local_handle_geometry import plane_base
        from scipy.spatial import cKDTree
        root=Path(root)
        events=[json.loads(line) for line in (root/'journal.jsonl').read_text().splitlines()]
        identities=[e for e in events if e.get('event')=='requested_identity_scores' and e.get('target')==target
                    and any(d.get('confidence',0)>=.8 for d in e.get('detections',[]))]
        if not identities:raise ValueError('previous_confirmed_requested_identity_required')
        decision=next(e['decision'] for e in reversed(events) if e.get('event')=='grasp_plan' and e['decision']['selected'] is not None)
        candidate=decision['selected'];stamp=candidate['observation_stamp_ns'];capture=None
        for path in (root/'captures').iterdir():
            snapshot_path=path/'snapshot.json'
            if not snapshot_path.exists():continue
            s=json.loads(snapshot_path.read_text())
            if s.get('streams',{}).get('depth',{}).get('stamp_ns')==stamp:
                capture=path;break
        if capture is None:raise ValueError('selected_candidate_capture_missing')
        item=dict(snapshot=s,depth=np.load(capture/'depth_m.npy'),
                  rgb=cv2.cvtColor(cv2.imread(str(capture/'rgb.png')),cv2.COLOR_BGR2RGB))
        plane=plane_base(s,measure(item['depth'],s,self.table)['plane_map'])
        points,colours,_=self._cloud(item);center=np.asarray(candidate['grasp_point_base_m'])
        height=points[:,2]-points[:,:2]@plane[:2]-plane[2]
        indices=np.flatnonzero((abs(points-center)<[.20,.20,.15]).all(1)&(height>.006)&(height<.4))
        _,ix=np.unique(np.floor(points[indices]/.003).astype(int),axis=0,return_index=True)
        indices=indices[ix];indices=indices[dominant_component(points[indices])]
        target_points=points[indices]
        distances=cKDTree(target_points).query(points)[0]
        near=(abs(points-center)<[.40,.40,.30]).all(1)
        environment=points[near&(distances>.006)]
        R,t=self._tf(s);self.template=target_points@R.T+t;self.colours=colours[indices]
        self.anchor=center@R.T+t;self.plane=plane;self.source_snapshot=s;self.latest_item=item
        self.identity_confirmed=True;self.target_name=target
        scene=Scene(target_points,environment,plane,stamp);scene.validate(self.policy)
        self.record(dict(event='previous_selected_plan_restored',source_run=str(root),capture=str(capture),
                         candidate=candidate,current_target_redetection_required=False,
                         historical_pose_explicitly_requested=True))
        return scene,decision

    def _sample(self):
        while True:
            item=self.rt.sample(force_new=True)
            stamp=int(item['snapshot']['streams']['depth']['stamp_ns'])
            if stamp>self.last_stamp:
                if self.self_geometry is not None:
                    from rclpy.time import Time
                    # RGBD can arrive before the matching arm TF (observed
                    # roughly 90 ms lag). Wait while stationary for that SAME
                    # stamp, rather than substituting an unrelated latest TF.
                    when=Time(nanoseconds=stamp)
                    while not all(self.rt.buf.can_transform('base_link',link,when) for link in self.self_geometry.links):
                        self.rt.pump(.02)
                    item['snapshot']['base_from_self_links_at_depth']={link:self.rt.tf_dict(
                        self.rt.buf.lookup_transform('base_link',link,when).transform)
                        for link in self.self_geometry.links}
                self.last_stamp=stamp;return item
            self.rt.pump(.03)

    @staticmethod
    def _tf(snapshot):
        from hand_camera_ray import rotation
        t=snapshot['tracking_transforms']['odom_from_base_at_depth']
        return rotation(t['quaternion']),np.asarray(t['translation'])

    def _cloud(self,item):
        cached=getattr(self,'_cloud_cache',None)
        if cached is not None and cached[0] is item:return cached[1]
        from round_body_geometry import grid
        from hand_camera_ray import rotation
        points=grid(item['depth'],item['snapshot'])
        valid=np.isfinite(item['depth'])&(item['depth']>.05)&(item['depth']<3)
        p=points[valid];rgb=item['rgb'][valid]
        palm=self.rt.palm_pose(item['snapshot']);R=rotation(palm['quaternion'])
        opening=float(item['snapshot']['joints']['hand_motor_joint'])
        parts=self.physical.sample(opening)['parts']
        boxes=[np.asarray(part['vertices_palm_m'])@R.T+palm['translation']
               for part in parts if not part.get('is_trigger')]
        # Rendered finger edges extend beyond collider faces. Include the
        # exported calibration uncertainty instead of assuming exact colliders
        # are the complete visible surface (which left robot pixels as obstacles).
        keep=outside_boxes(p,boxes,margin=max(.004,self.hand.uncertainty+.004))
        transforms=item['snapshot'].get('base_from_self_links_at_depth')
        if transforms and self.self_geometry is not None:
            keep &= outside_boxes(p,self.self_geometry.boxes(transforms),margin=.004)
        indices=np.flatnonzero(valid)[keep]
        result=p[keep],rgb[keep],indices
        self._cloud_cache=(item,result)
        return result

    def observe_stopped(self,target):
        from object_grasp_profiles import apply as apply_object_grasp_profile
        apply_object_grasp_profile(self,target)
        from .support_observation_recovery import observe
        return observe(self,target,self._observe_stopped_once)

    def _observe_stopped_once(self,target):
        from local_handle_geometry import plane_base
        from can_destination_surface import measure
        if getattr(self,'identity_confirmed',False) and target==self.target_name:
            self.pose_locked=False  # A new candidate requires a new observation.
            # Retain established identity while checking NEW measured geometry;
            # do not require the detector to recognise a hand-occluded object
            # again merely because a small alignment correction was requested.
            from scipy.spatial import cKDTree
            item,m=self._observe_tracking();s=item['snapshot'];R,t=self._tf(s)
            points,_,_=self._cloud(item);model=(self.template-t)@R
            near=(points>=model.min(0)-.3).all(1)&(points<=model.max(0)+.3).all(1)
            environment=points[near&(cKDTree(model).query(points)[0]>.008)]
            plane=self._current_plane(s)
            self.plane=plane;self.source_snapshot=s
            # Registration matches are often only one visible strip. Keep the
            # registered full reference for shape selection; actual closure
            # still checks only this frame's observed_contact_points.
            self.scene=Scene(model,environment,plane,self.last_stamp)
            self.record(dict(event='identity_retained_with_new_surface_observation',target=target,
                             matched_points=len(self.observed_contact_points)))
            self.baseline=None;self.lift_done=False;self.evidence=None
            return self.scene
        self.target_name=target
        item=self._sample();self.serial+=1
        path=self.rt.save(item,'identity-'+str(self.serial))
        raw=self.worker.infer(path,pump=lambda:self.rt.pump(0))
        from apple_transfer_automation import write_json
        write_json(path/'detections.json',raw)
        self.record(dict(event='requested_identity_scores',target=target,
                         detections=[d for d in raw['detections'] if d['name']==target]))
        from edge_view import recenter_head
        snap=item['snapshot'];j=snap['joints']
        self.identity_head_hint=recenter_head([d for d in raw['detections'] if d['name']==target],
            item['depth'].shape,(j['head_pan_joint'],j['head_tilt_joint']),snap['rgb_k'])
        from .identity_policy import confidence_passes,within_source,identity_component,source_association
        constrained=getattr(self,'source_constrained',False)
        rows=[d for d in raw['detections'] if d['name']==target and confidence_passes(d['confidence'],constrained)]
        if not rows or (not constrained and len(rows)!=1):
            raise AttemptFailure(Failure.OCCLUDED,'requested_identity_not_unique_in_current_image')
        s=item['snapshot'];plane=plane_base(s,measure(item['depth'],s,self.table)['plane_map'])
        points,rgb,indices=self._cloud(item)
        v,u=np.unravel_index(indices,item['depth'].shape)
        height=points[:,2]-points[:,:2]@plane[:2]-plane[2]
        selected=None
        for row in sorted(rows,key=lambda d:d['confidence'],reverse=True):
            x1,y1,x2,y2=row['box']
            mask=(u>=x1)&(u<x2)&(v>=y1)&(v<y2)&(height>.006)&(height<.4)
            candidate=np.flatnonzero(mask)
            from .foreground_depth import foreground_mask
            depth_values=item['depth'].ravel()[indices[candidate]]
            foreground=foreground_mask(depth_values)
            if not foreground.all():
                self.record(dict(event='grasp_background_depth_separated',input_points=len(candidate),foreground_points=int(foreground.sum()),identity_rejected=False))
            candidate=candidate[foreground]
            try:
                _,ix=np.unique(np.floor(points[candidate]/.003).astype(int),axis=0,return_index=True)
                candidate=candidate[ix]
                component,waived=identity_component(points[candidate],row['confidence'])
                candidate=candidate[component]
                if waived:self.record(dict(event='identity_depth_connectivity_check_disabled',
                    confidence=row['confidence'],measured_points=len(candidate),depth_fabricated=False))
            except UnobservedSurface as exc:
                self.record(dict(event='identity_point_cloud_rejected',reason=str(exc),confidence=row['confidence']))
                continue
            if constrained:
                from apple_transfer_auto_plan import map_point
                center=map_point(s,np.median(points[candidate],axis=0))
                accepted_source=source_association(center,self.table,row['confidence'])
                self.record(dict(event='source_association_check',center_map_m=list(center),furniture=self.table['id'],confidence=row['confidence'],accepted=accepted_source,strict_inside=within_source(center,self.table),high_confidence_margin_m=.15))
                if not accepted_source:continue
            selected=candidate
            self.identity_preview=dict(snapshot=s,points=points[selected],confidence=row['confidence'])
            self.record(dict(event='source_identity_accepted',confidence=row['confidence'],
                source_constrained=constrained,source_table=self.table['id'],
                threshold=.5 if constrained else .8,source_location_checked=constrained))
            break
        if selected is None:
            raise AttemptFailure(Failure.OCCLUDED,'no_target_depth_component_in_requested_source')
        # A clipped identity box cannot establish the bottle's narrow region.
        self.identity_head_hint=recenter_head([row],item['depth'].shape,
            (s['joints']['head_pan_joint'],s['joints']['head_tilt_joint']),s['rgb_k'])
        if self.identity_head_hint is not None:
            self.record(dict(event='target_geometry_clipped_recenter_required',box=row['box'],head_goal=self.identity_head_hint))
            raise AttemptFailure(Failure.OCCLUDED,'target_geometry_clipped_recenter_required')
        target_points=points[selected];colours=rgb[selected]
        # All other measured points, including disconnected points IN the box,
        # remain obstacles. Exclude only points near the selected component.
        from scipy.spatial import cKDTree
        dist=cKDTree(target_points).query(points)[0]
        near=(points>=target_points.min(0)-[.30,.30,.15]).all(1)&(points<=target_points.max(0)+[.30,.30,.20]).all(1)
        environment=points[near&(dist>.006)]
        R,t=self._tf(s)
        self.template=target_points@R.T+t;self.colours=colours
        self.anchor=np.median(self.template,axis=0)
        self.identity_confirmed=True
        self.plane=plane;self.source_snapshot=s
        self.identity_head_goal=(s['joints']['head_pan_joint'],s['joints']['head_tilt_joint'])
        self.latest_item=item
        self.scene=Scene(target_points,environment,plane,self.last_stamp)
        self.candidate=None;self.baseline=None;self.lift_done=False;self.evidence=None
        return self.scene

    def _observe_tracking(self,*,lift_delta=None):
        if self.pose_locked and lift_delta is None:
            return self._observe_locked()
        item=self._sample();s=item['snapshot'];R,t=self._tf(s)
        points,rgb,_=self._cloud(item)
        template=self.template+(np.zeros(3) if lift_delta is None else lift_delta)
        model=(template-t)@R
        mask=(points>=model.min(0)-.035).all(1)&(points<=model.max(0)+.035).all(1)
        try:result=register_surface(model,self.colours,points[mask],rgb[mask],
            require_surface_extent=not getattr(self,"skip_tiny_surface_extent",False),
            max_shift=None if getattr(self,"unlimited_registration_shift",False) else .035)
        except UnobservedSurface as exc:
            self.serial+=1
            path=self.rt.save(item,'rejected-track-'+str(self.serial))
            np.savez_compressed(path/'registration-inputs.npz',template=model,template_rgb=self.colours,
                                observed=points[mask],observed_rgb=rgb[mask])
            from .static_anchor import allowed,retain
            if allowed(self,str(exc),lift_delta):return retain(self,str(exc))
            raise AttemptFailure(Failure.OCCLUDED,str(exc)) from exc
        shift=R@result.translation
        self.observed_contact_points=result.matched_points
        self.observed_contact_stamp=self.last_stamp
        anchor=self.anchor+(np.zeros(3) if lift_delta is None else lift_delta)+shift
        if lift_delta is None:
            self.template+=shift;self.anchor=anchor
        center=(anchor-t)@R
        measurement=dict(center_m=center.tolist(),stamp_ns=self.last_stamp,
            position_source='measured_colour_geometry_registration',depth_measured=True,
            matched_surface_points=len(result.matched_points),
            surface_residual_p90_m=result.residual_p90_m,
            template_coverage=result.template_coverage)
        self.last_tracking_measurement=measurement
        self.rt.fresh_measurement(s)
        self.serial+=1;self.rt.save(item,'tracked-'+str(self.serial),measurement)
        self.latest_item=item
        return item,measurement

    def _observe_locked(self):
        item=self._sample();s=item['snapshot'];R,t=self._tf(s)
        measurement=dict(center_m=((self.anchor-t)@R).tolist(),stamp_ns=self.last_stamp,
            position_source='locked_selected_pose_propagated_with_robot_feedback',
            depth_measured=False,current_target_observation_required=False,
            reference_stamp_ns=self.source_snapshot['streams']['depth']['stamp_ns'])
        self.rt.fresh_measurement(s)
        self.serial+=1;self.rt.save(item,'locked-'+str(self.serial),measurement)
        self.latest_item=item
        return item,measurement

    def _check_open_path(self,goals,*,translation=None,allow_target_contact=False,table_margin=None):
        """Sample the entire hand against observed target/environment and table.

        The measured support extent restricts plane checks; do not treat the
        tabletop as an infinite obstacle outside its observed area.
        """
        item=self.latest_item
        cached=getattr(self,'_path_cache',None)
        if cached is not None and cached[0] is item:
            plane,hull,obstacles=cached[1]
        else:
            from scipy.spatial import ConvexHull
            s=item['snapshot'];R,t=self._tf(s)
            if self.pose_locked:
                points=(self.locked_environment_world-t)@R
            else:
                points,_,_=self._cloud(item)
            model=(self.template-t)@R
            R0,t0=self._tf(self.source_snapshot)
            n0=np.r_[-self.plane[:2],1.];nw=R0@n0
            c=self.plane[2]+nw@t0;n=R.T@nw
            plane=np.r_[-n[:2]/n[2],(c-nw@t)/n[2]]
            support=points[abs(points[:,2]-points[:,:2]@plane[:2]-plane[2])<.008]
            support=support[(abs(support[:,:2]-np.median(model[:,:2],axis=0))<.65).all(1)]
            hull=ConvexHull(support[:,:2]) if len(support)>10 else None
            if hull is None:raise AttemptFailure(Failure.OCCLUDED,'support_extent_not_observed')
            obstacles=points[points[:,2]>points[:,:2]@plane[:2]+plane[2]+.008]
            _,ix=np.unique(np.floor(obstacles/.003).astype(int),axis=0,return_index=True)
            obstacles=obstacles[ix]
            self._path_cache=(item,(plane,hull,obstacles))
        for goal in goals:
            palm,orient=forward(goal)
            if translation is not None:palm+=translation
            boxes=self.hand.open@orient.T+palm
            hits=~outside_boxes(obstacles,boxes,margin=.001)
            if allow_target_contact:
                from .target_contact import permitted_target_contact
                R,t=self._tf(item['snapshot'])
                target=(self.template-t)@R
                hits &= ~permitted_target_contact(obstacles,target,palm,orient,self.hand.contacts)
            if np.count_nonzero(hits)>3:
                self.serial+=1
                path=self.rt.save(item,'rejected-path-'+str(self.serial))
                np.savez_compressed(path/'collision-inputs.npz',points=obstacles[hits],boxes=boxes)
                self.record(dict(event='open_path_collision_evidence',capture=str(path),
                    points=int(hits.sum()),bounds=[obstacles[hits].min(0).tolist(),obstacles[hits].max(0).tolist()],
                    goal=list(map(float,goal)),translation=None if translation is None else list(map(float,translation))))
                raise AttemptFailure(Failure.APPROACH_COLLISION,'open_hand_path_intersects_observed_surface')
            from .table_clearance import hand_table_clearance
            clearance=hand_table_clearance(palm,orient,plane,self.hand,hull)
            margin=self.policy.table_margin_m if table_margin is None else table_margin
            if clearance<margin-1e-6:
                self.record(dict(event='open_path_table_clearance_rejected',clearance_m=clearance,required_m=margin))
                exc=AttemptFailure(Failure.APPROACH_COLLISION,'open_hand_path_too_low_for_table')
                exc.clearance_m=clearance;exc.required_m=margin
                raise exc

    def _recover_table_height(self,exc):
        from .height_recovery import recovery_raise
        dz=recovery_raise(exc)
        if dz is None:raise exc
        goal=np.asarray(self.rt.state()['joints'],float).copy();goal[0]+=dz
        if goal[0]>self.policy.max_lift_m-self.policy.lift_check_m:raise exc
        # Permit the existing clearance only during pure upward escape; endpoint
        # and the original motion must still pass the full normal checks.
        self._preclose_start_clearance=exc.clearance_m
        self._arm(goal,'side-preclose-camera-height')
        self.record(dict(event='approach_table_height_recovered',raise_m=dz,
                         old_clearance_m=exc.clearance_m,required_m=exc.required_m))
        return dz

    def preclose_height_delta(self,error,snapshot):
        from .planner import _minimum_palm_height
        R,t=self._tf(snapshot);R0,t0=self._tf(self.source_snapshot)
        nw=R0@np.r_[-self.plane[:2],1.];n=R.T@nw
        plane=np.r_[-n[:2]/n[2],(self.plane[2]+nw@t0-nw@t)/n[2]]
        palm,orient=forward(self.rt.state()['joints'])
        minimum=max(_minimum_palm_height(palm[:2]+f*np.asarray(error[:2]),orient,plane,self.hand,self.policy)
                    for f in (0.,1.))
        requested=float(error[2]) if abs(error[2])>.002 else 0.
        self._preclose_start_clearance=palm[2]-_minimum_palm_height(palm[:2],orient,plane,self.hand,self.policy)+self.policy.table_margin_m
        return max(requested,minimum+.001-palm[2])

    def _arm(self,goal,label):
        start=np.asarray(self.rt.state()['joints']);goal=np.asarray(goal,float)
        count=max(2,int(np.max(abs(goal[:5]-start[:5]))/.04)+1)
        contact=label in ('generic-lower-to-contact-height','rotation_retry_complete_contact_height','side-preclose-camera-height') or label.startswith('rotation_only_retry_')
        margin=None
        if label=='side-preclose-camera-height' and goal[0]>start[0] and np.allclose(goal[1:5],start[1:5]):
            margin=min(self.policy.table_margin_m,getattr(self,'_preclose_start_clearance',self.policy.table_margin_m))
        try:
            self._check_open_path(np.linspace(start,goal,count),allow_target_contact=contact,table_margin=margin)
        except AttemptFailure as exc:
            if not label.startswith('rotation_only_retry_'):raise
            dz=self._recover_table_height(exc)
            start=np.asarray(self.rt.state()['joints']);goal[0]+=dz
            self._check_open_path(np.linspace(start,goal,count),allow_target_contact=contact)
        if margin is not None:self._check_open_path([goal],allow_target_contact=True)
        self.rt.move_joints(goal.tolist(),label)
        self.retreat_steps.append(('arm',start.tolist(),goal.tolist()))

    def _base(self,delta,label):
        arm=self.rt.state()['joints']
        count=max(2,int(np.linalg.norm(delta[:2])/.01)+1)
        def check():
            for f in np.linspace(0,1,count):
                self._check_open_path([arm],translation=np.r_[np.asarray(delta[:2])*f,0.],
                    allow_target_contact=label in ('generic-final-straight-approach','rotation_retry_complete_alignment','generic-near-measured-alignment','side-preclose-camera-alignment'))
        try:check()
        except AttemptFailure as exc:
            self._recover_table_height(exc)
            arm=self.rt.state()['joints']
            check()
        self.rt.base_move(delta,arm[:5],label)
        self.retreat_steps.append(('base',list(delta),None))

    def execute_candidate(self,candidate,scene):
        self.contact_attempt_started=False
        self.preextension_anchor_confirmed=False
        self.static_precontact_anchor=False
        if self.holding_state()!='empty':raise AttemptFailure(Failure.HOLD_UNCERTAIN)
        self.candidate=candidate;self.retreat_steps=[]
        R,t=self._tf(self.source_snapshot)
        self.anchor=np.asarray(candidate['grasp_point_base_m'])@R.T+t
        if self.policy.lock_selected_pose:
            points,_,_=self._cloud(self.latest_item)
            self.locked_environment_world=points@R.T+t
            self.pose_locked=True
            self._path_cache=None
            self.record(dict(event='selected_grasp_pose_locked',candidate=candidate,
                execution_basis='initial_measured_scene_and_current_robot_feedback',
                target_occlusion_does_not_block_approach_or_closure=True))
        self.holding='empty';self.closed=False
        # Refresh while the arm is still tucked and before it covers the object.
        was_locked=self.pose_locked;self.pose_locked=False
        try:self._observe_tracking()
        finally:self.pose_locked=was_locked
        self.preextension_anchor_confirmed=True
        self.static_precontact_anchor=True
        self.record(dict(event='preextension_anchor_confirmed',depth_measured=True))
        # Raise first, orient clear of the surface, then approach. Final height
        # comes from the full-hand candidate, never a per-class centre fraction.
        goal=list(candidate['arm_goal'])
        raised=min(.69,goal[0]+.08)
        start=self.rt.state()['joints'].copy();start[0]=max(start[0],raised)
        goal[0]=start[0]
        from .combined_pregrasp import prepare_arm
        prepare_arm(self,start,goal)
        self.rt.face_source((0.,-.65 if candidate['strategy']=='side' else -.8))
        item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
        # One measured coarse move, not centimetre-by-centimetre nudging.
        if candidate['strategy']=='side':
            if self.policy.sequential_side and abs(error[1])>.004:
                if error[0]<.15:
                    self._base([float(error[0]-.15),0.,0.],'template-clear-before-lateral')
                    item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                self._base([0.,float(error[1]),0.],'template-align-finger-midline')
                item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                if abs(error[1])>.006:
                    raise AttemptFailure(Failure.MISALIGNED,'template_lateral_alignment_requires_correction')
            desired=[float(error[0]-.12),0. if self.policy.sequential_side else float(error[1]),0.]
        else:desired=[float(error[0]),float(error[1]),0.]
        self._base(desired,'generic-measured-approach')
        # The far-view anchor cannot certify actual alignment after a long
        # approach. Refresh geometry once while the open hand is still raised;
        # retain established identity, then lock again for the occluded descent.
        was_locked=self.pose_locked
        from .contact_observation import observe_current
        item,m=observe_current(self)
        error=self.rt.contact_error(m['center_m'],item['snapshot'])
        self.record(dict(event='near_contact_geometry_refreshed',error_base_m=error.tolist(),measurement=m))
        if was_locked:
            R,t=self._tf(item['snapshot'])
            points,_,_=self._cloud(item)
            self.locked_environment_world=points@R.T+t
            self._path_cache=None
        if candidate['strategy']=='side':
            from .near_side_plan import replan_side
            R,t=self._tf(item['snapshot'])
            model=(self.template-t)@R
            points,_,_=self._cloud(item)
            R0,t0=self._tf(self.source_snapshot);nw=R0@np.r_[-self.plane[:2],1.]
            n=R.T@nw;c=self.plane[2]+nw@t0
            plane=np.r_[-n[:2]/n[2],(c-nw@t)/n[2]]
            current_target=None
            if getattr(self,'observed_contact_stamp',None)==item['snapshot']['streams']['depth']['stamp_ns']:
                current_target=self.observed_contact_points
            decision=replan_side(model,points,plane,self.last_stamp,self.hand,self.policy,
                                 observed_target=current_target)
            fresh=decision['selected']
            self.record(dict(event='near_side_contact_replanned',candidate=fresh,
                rejected=decision.get('rejected',{}),basis='current_geometry_before_descent',
                current_target_points=0 if current_target is None else len(current_target)))
            if fresh is None:raise AttemptFailure(Failure.NO_CANDIDATE,'near_side_geometry_has_no_clear_contact')
            candidate=fresh;self.candidate=fresh
            self.anchor=np.asarray(fresh['grasp_point_base_m'])@R.T+t
            item,m=self._observe_locked();error=self.rt.contact_error(m['center_m'],item['snapshot'])
        near_delta=[float(error[0]-(.12 if candidate['strategy']=='side' else 0.)),float(error[1]),0.]
        if np.linalg.norm(near_delta[:2])>.004:
            self._base(near_delta,'generic-near-measured-alignment')
        item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
        if abs(error[1])>.006 or (candidate['strategy']=='overhead' and abs(error[0])>.009):
            raise AttemptFailure(Failure.MISALIGNED,'new_observation_requires_pose_correction')
        goal=self.rt.state()['joints'].copy();goal[0]+=float(error[2])
        self._arm(goal,'generic-lower-to-contact-height')
        item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
        if candidate['strategy']=='side':
            self._base([float(error[0]),0.,0.],'generic-final-straight-approach')
        item,m=self._observe_tracking()
        self.contact_attempt_started=True
        self._finish_grasp(item,m)

    def _observe_for_closure(self):
        locked=self.pose_locked;static=getattr(self,'static_precontact_anchor',False)
        self.pose_locked=False;self.static_precontact_anchor=False
        try:
            item,m=self._observe_tracking()
            if m.get('depth_measured') is not True:
                raise AttemptFailure(Failure.OCCLUDED,'closure_requires_current_target_depth')
            return item,m
        finally:
            self.pose_locked=locked;self.static_precontact_anchor=static

    def try_aligned_trial_before_rotation(self,reason):
        # Only missing contact depth qualifies; width/collision/pose failures do not.
        if not getattr(self,'trial_before_rotation_enabled',False):return False
        if reason!='contact_band_unobserved' or self.holding_state()!='empty':return False
        if getattr(self,'candidate',{}).get('strategy')!='side':return False
        self.check_task_cancelled()
        self.record(dict(event='aligned_trial_before_rotation',wrist_rotation_sent=False))
        # Fresh RGBD alignment and all existing contact/path/closure checks remain.
        self._finish_grasp(None,None)
        return True

    def _finish_grasp(self,item,m):
        candidate=self.candidate
        from .preclose_alignment import align_before_close
        item,m=align_before_close(self)
        fit=self._actual_contact_check(item['snapshot'])
        from .contact import small_height_correction
        correction=small_height_correction(fit,self.policy.table_margin_m)
        if correction is not None:
            goal=list(self.rt.state()['joints']);goal[0]+=correction
            if goal[0]<=self.policy.max_lift_m-self.policy.lift_check_m:
                self._check_open_path([goal],allow_target_contact=True)
                self.rt.move_joints(goal,'preclose-small-height-correction')
                self.anchor[2]+=correction
                item,m=self._observe_tracking()
                fit=self._actual_contact_check(item['snapshot'])
                self.record(dict(event='preclose_height_corrected_locally',raise_m=correction,fit=fit))
        self.record(dict(event='preclose_contact_decision',
                         error_base_m=self.rt.contact_error(m['center_m'],item['snapshot']).tolist(),**fit))
        if not fit['fits']:
            raise AttemptFailure(Failure.MISALIGNED,fit['reason'])
        # Full closing sweep clearance was validated in the plan; final current
        # material fit is checked again using actual reobserved target position.
        R,t=self._tf(item['snapshot']);current=(self.template-t)@R
        R0,t0=self._tf(self.source_snapshot);n0=np.r_[-self.plane[:2],1.]
        nw=R0@n0;n=R.T@nw;c=self.plane[2]+nw@t0
        plane=np.r_[-n[:2]/n[2],(c-nw@t)/n[2]]
        check_scene=Scene(current,np.empty((0,3)),plane,self.last_stamp)
        row=dict(strategy=candidate['strategy'],angle=candidate['wrist_roll_rad'],center=np.asarray(m['center_m']))
        checked,reason=evaluate(row,check_scene,self.hand,self.policy)
        if checked is None:raise AttemptFailure(Failure.MISALIGNED,reason)
        if np.linalg.norm(np.asarray(checked['grasp_point_base_m'])-m['center_m'])>.004:
            raise AttemptFailure(Failure.MISALIGNED,'new_clearance_height_requires_replan')
        fit=self._actual_contact_check(item['snapshot'])
        self.record(dict(event='actual_contact_fit',**fit))
        if not fit['fits']:raise AttemptFailure(Failure.MISALIGNED,fit['reason'])
        self.static_precontact_anchor=False
        self.holding='uncertain'
        try:
            closure,_,_=self.rt.close_gripper(self._observe_for_closure,self.rt.state()['joints'][:5])
        except AttemptFailure:
            # The observation callback runs before publishing the close command.
            self.holding='empty'
            raise
        except (ValueError,RuntimeError) as exc:
            text=str(exc)
            if 'without_contact' in text:
                self.holding='empty';self.closed=True
                raise AttemptFailure(Failure.EMPTY_CLOSE,text) from exc
            if 'not_aligned' in text:
                self.holding='empty'
                raise AttemptFailure(Failure.MISALIGNED,text) from exc
            raise
        self.closed=True;self.opening=float(closure['closure_evidence']['observed_opening_rad'])
        self.record(dict(event='grasp_closure_confirmed',basis='stable_nonempty_joint_opening',
                         pressure_measured=False,opening_rad=self.opening,grasp_verified=False))
        item,m=self._observe_tracking()
        R,t=self._tf(item['snapshot']);palm=self.rt.palm_pose(item['snapshot'])
        # Capture the source support level before lifting; do not invent a full
        # object mesh from a partially visible surface.
        from support_reference import make_reference
        from hand_camera_ray import rotation
        current_points=(self.template-t)@R
        try:
            self.source_support_reference=make_reference(current_points,self._current_plane(item['snapshot']),
                palm['translation'],rotation(palm['quaternion']),item['snapshot']['streams']['depth']['stamp_ns'])
        except ValueError as exc:
            self.source_support_reference=None
            self.record(dict(event='source_support_reference_unavailable',reason=str(exc)))
        self.pre_lift_snapshot=item['snapshot']
        self.baseline=dict(anchor=self.anchor.copy(),template=self.template.copy(),
                           palm=np.asarray(palm['translation'])@R.T+t,
                           object_position_basis='locked_reference' if self.pose_locked else 'current_surface_measurement')

    def return_uncertain_to_source(self):
        """Lower to the measured pre-lift source support before opening, then clear."""
        before=getattr(self,'pre_lift_snapshot',None)
        if before is None or self.source_support_reference is None:raise ValueError('source_support_for_recovery_missing')
        self.rt.face_source((0.,-.8))
        item=self._sample();current=item['snapshot']
        R0,t0=self._tf(before);R,t=self._tf(current)
        if np.linalg.norm(t[:2]-t0[:2])>.04:raise ValueError('base_moved_since_source_lift')
        names=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint']
        goal=self.rt.state()['joints'].copy()
        if max(abs(goal[i]-before['joints'][names[i]]) for i in range(1,5))>.04:raise ValueError('source_recovery_orientation_changed')
        from can_destination_surface import measure
        from local_handle_geometry import plane_base
        plane=plane_base(current,measure(item['depth'],current,self.table)['plane_map'])
        reference=self._current_plane(current)
        xy=((self.anchor-t)@R)[:2]
        if abs(np.r_[xy,1.]@(plane-reference))>.02:raise ValueError('source_support_changed')
        goal[0]=float(before['joints']['arm_lift_joint'])
        self.rt.move_joints(goal,'return_uncertain_grasp_to_source_support')
        goal=self.rt.state()['joints'].copy();goal[5]=.65
        self.rt.move_joints(goal,'release_uncertain_on_source_support',hand=True)
        self.holding='empty';self.closed=False;self.pose_locked=False;self.identity_confirmed=False
        from withdrawal_observer import WithdrawalObserver
        from table_withdrawal import TableWithdrawal
        motion=getattr(self,'session_motion',None)
        if motion is None:raise ValueError('source_recovery_motion_adapter_missing')
        TableWithdrawal(self.rt,motion,WithdrawalObserver(self,self.table,lambda:None),label='uncertain-source-recovery-'+str(self.serial))()
        self.rt.prepare_navigation_from_current()
        head=getattr(self,'identity_head_goal',(0.,0.))
        if getattr(self.policy,'template_family','')=='neck_bottle':
            index=getattr(self,'neck_view_retry',0)
            head=(0.,(-.15,-.30,-.50,0.)[index%4]);self.neck_view_retry=index+1
            self.record(dict(event='neck_view_retry',attempt=index+1,head_goal=head))
        self.rt.face_source(head)
        self.record(dict(event='uncertain_grasp_returned_to_measured_source',holding='empty',automatic_regrasp_allowed=True))

    def _sample_for_path(self):self.latest_item=self._sample()

    def verify_lift(self):
        """Lift then reclose: fresh blocked-jaw feedback is the success policy."""
        import time
        from canned_grasp_runtime import TOPICS
        from .grip_confirmation import RecloseEvidence
        if self.evidence is not None:return self.evidence
        if self.baseline is None:return LiftEvidence(False,False,False,0)
        if not self.lift_done:
            goal=self.rt.state()['joints'].copy()
            from .lift_headroom import lift_goal
            goal,distance=lift_goal(goal,self.policy)
            self.record(dict(event='verification_lift_headroom',distance_m=distance,minimum_m=min(.05,self.policy.lift_check_m)))
            self.rt.move_joints(goal,'generic-verification-lift');self.lift_done=True
        self.rt.sample(profile='state');state=self.rt.state()
        lifted=float(state['joints'][0]-self.pre_lift_snapshot['joints']['arm_lift_joint'])
        minimum=min(.05,self.policy.lift_check_m)
        if lifted+1e-6<minimum:raise ValueError('verification_lift_not_reached')
        topic=TOPICS[3];pub=self.rt.publisher(topic);gate=RecloseEvidence()
        try:
            self.rt.ownership(topic)
            pub.publish(self.rt.trajectory(['hand_motor_joint'],[0.],6.))
            self.record(dict(event='lift_reclose_sent',lift_distance_m=lifted,visual_confirmation_required=False))
            sent=time.monotonic();decision=None
            while time.monotonic()-sent<11:
                self.rt.pump(.02);self.rt.ownership(topic);state=self.rt.state()
                if time.monotonic()-sent<6.:continue
                decision=gate.observe(state['stamp_ns'],float(state['joints'][5]))
                if decision is not None:break
            if decision is not True:
                if decision is False:
                    self.holding='empty';self.record(dict(event='lift_reclose_empty',opening_rad=float(state['joints'][5])))
                    raise AttemptFailure(Failure.EMPTY_CLOSE,'post_lift_closed_without_object')
                raise AttemptFailure(Failure.HOLD_UNCERTAIN,'post_lift_joint_feedback_unstable')
        finally:self.rt.node.destroy_publisher(pub)
        self.opening=float(state['joints'][5]);self.holding='held'
        self.evidence=LiftEvidence(True,False,True,len(gate.rows),'lift_and_reclose_joint_feedback',lifted,True,minimum)
        # Geometry remains the pre-lift measured surface in palm coordinates.
        # Joint confirmation does not invent a new visual measurement.
        from .held_context import build_context
        from hand_camera_ray import rotation
        pre=self.pre_lift_snapshot;R,t=self._tf(pre);palm=self.rt.palm_pose(pre)
        self.held_context=build_context(points_base=(self.template-t)@R,
            palm_translation_base=palm['translation'],palm_rotation_base=rotation(palm['quaternion']),
            stamp_ns=int(pre['streams']['depth']['stamp_ns']),target=self.target_name,
            source_table=self.table['id'],joints=state['joints'],lift_evidence=vars(self.evidence),surface_rgb=self.colours)
        self.held_context.update(geometry_source='prelift_measured_surface_assumed_retained_in_palm',
            current_surface_visually_verified=False,source_support_reference=getattr(self,'source_support_reference',None))
        self.record(dict(event='generic_lift_verified',evidence=vars(self.evidence),opening_rad=self.opening,pressure_measured=False))
        return self.evidence

    def _verify_lift_current_view(self):
        if self.evidence is not None:return self.evidence
        if self.baseline is None:return LiftEvidence(False,False,False,0)
        if not self.lift_done:
            goal=self.rt.state()['joints'].copy()
            if not grip_retained(goal[5],self.opening):
                self.record(dict(event='grasp_confirmation_failed',reason='opening_changed_before_lift'))
                return LiftEvidence(False,False,False,0)
            goal[0]+=self.policy.lift_check_m
            self.rt.move_joints(goal,'generic-verification-lift');self.lift_done=True
        rows=[]
        last_stamp=int(self.pre_lift_snapshot['streams']['depth']['stamp_ns']) if getattr(self,'pre_lift_snapshot',None) else 0
        for _ in range(3):
            item=self._sample();R,t=self._tf(item['snapshot'])
            palm=np.asarray(self.rt.palm_pose(item['snapshot'])['translation'])@R.T+t
            delta=palm-self.baseline['palm']
            try:
                observed,m=self._observe_tracking(lift_delta=delta)
            except AttemptFailure:
                retained=grip_retained(self.rt.state()['joints'][5],self.opening)
                return LiftEvidence(False,False,retained,len(rows))
            R,t=self._tf(observed['snapshot']);actual=np.asarray(m['center_m'])@R.T+t-self.baseline['anchor']
            # Compare object and palm from the SAME depth snapshot, not the
            # preliminary frame used to seed registration.
            palm=np.asarray(self.rt.palm_pose(observed['snapshot'])['translation'])@R.T+t
            delta=palm-self.baseline['palm']
            stamp=int(observed['snapshot']['streams']['depth']['stamp_ns'])
            retained=grip_retained(observed['snapshot']['joints']['hand_motor_joint'],self.opening)
            if not lift_sample_passes(actual,delta,retained,stamp,last_stamp):
                return LiftEvidence(False,False,retained,len(rows))
            last_stamp=stamp
            rows.append(dict(stamp_ns=stamp,grip_retained=retained,object_delta_m=actual.tolist(),palm_delta_m=delta.tolist(),measurement=m))
        self.holding='held';self.evidence=LiftEvidence(True,True,True,len(rows))
        # Same final tracked depth frame as the successful lift observation.
        from .held_context import build_context
        from hand_camera_ray import rotation
        palm=self.rt.palm_pose(observed['snapshot'])
        if self.observed_contact_stamp != observed['snapshot']['streams']['depth']['stamp_ns']:
            raise ValueError('held_surface_snapshot_mismatch')
        from scipy.spatial import cKDTree
        cloud,colours,_=self._cloud(observed)
        distances,colour_indices=cKDTree(cloud).query(self.observed_contact_points)
        if np.max(distances)>.0001:
            raise ValueError('held_surface_colour_frame_mismatch')
        self.held_context=build_context(
            points_base=self.observed_contact_points,
            palm_translation_base=palm['translation'],
            palm_rotation_base=rotation(palm['quaternion']),
            stamp_ns=int(self.observed_contact_stamp),target=self.target_name,
            source_table=self.table['id'],joints=self.rt.state()['joints'],
            lift_evidence=vars(self.evidence),surface_rgb=colours[colour_indices])
        self.held_context['source_support_reference']=getattr(self,'source_support_reference',None)
        self.record(dict(event='generic_lift_verified',evidence=rows))
        return self.evidence

    def check_task_cancelled(self):
        pass # StageBindings installs the current operation cancellation token.

    def begin_rotation_cycle(self):
        self.check_task_cancelled()
        self.rotation_cycle_joints=self.rt.state()['joints'].copy()
        self.rotation_cycle_candidate=dict(self.candidate)

    def rotate_only_retry(self,attempt):
        """Rotate first; finish interrupted approach before attempting closure."""
        self.check_task_cancelled()
        if self.holding_state()!='empty':raise AttemptFailure(Failure.HOLD_UNCERTAIN)
        goal=self.rt.state()['joints'].copy()
        if goal[5]<.60:
            goal[5]=.65;self.rt.move_joints(goal,'rotation_retry_open_empty',hand=True)
        # Undo only the standard verification lift if the object slipped.
        if self.lift_done and getattr(self,'pre_lift_snapshot',None):
            goal=self.rt.state()['joints'].copy();goal[0]=self.pre_lift_snapshot['joints']['arm_lift_joint']
            self._arm(goal,'rotation_retry_restore_pretest_height')
        offsets=(math.pi/6,-math.pi/6,math.pi/3,-math.pi/3)
        goal=self.rt.state()['joints'].copy()
        goal[4]=(self.rotation_cycle_joints[4]+offsets[attempt-1]+math.pi/2)%math.pi-math.pi/2
        self._sample_for_path()
        self._arm(goal,'rotation_only_retry_'+str(attempt))
        self.candidate=dict(self.rotation_cycle_candidate,wrist_roll_rad=goal[4])
        self.baseline=None;self.evidence=None;self.lift_done=False;self.closed=False
        if not getattr(self,'contact_attempt_started',True):
            self._complete_approach_after_rotation()
        item,m=self._observe_tracking()
        self._finish_grasp(item,m)

    def _complete_approach_after_rotation(self):
        """Resume toward the same measured anchor, never close at raised height.

        This is completion of an interrupted approach, not a new grasp point or
        a retreat. Failures propagate to the next of the four wrist attempts.
        """
        self.check_task_cancelled()
        goal=self.rt.state()['joints'].copy()
        goal[1:4]=self.candidate['arm_goal'][1:4]
        self._arm(goal,'rotation_retry_restore_approach_posture')
        item,m=self._observe_tracking()
        error=self.rt.contact_error(m['center_m'],item['snapshot'])
        if np.linalg.norm(error[:2])>.004:
            self._base([float(error[0]),float(error[1]),0.], 'rotation_retry_complete_alignment')
        item,m=self._observe_tracking()
        error=self.rt.contact_error(m['center_m'],item['snapshot'])
        goal=self.rt.state()['joints'].copy();goal[0]+=float(error[2])
        self._arm(goal,'rotation_retry_complete_contact_height')
        self.contact_attempt_started=True

    def reset_rotation_cycle(self):
        """After four rotations: withdraw, fold, and reacquire current target."""
        self.check_task_cancelled()
        if self.holding_state()!='empty':raise AttemptFailure(Failure.HOLD_UNCERTAIN)
        if abs(self.rt.state()['joints'][1])>.05:
            goal=self.rt.state()['joints'].copy()
            if goal[5]<.60:
                goal[5]=.65;self.rt.move_joints(goal,'cycle_open_empty',hand=True)
            from withdrawal_observer import WithdrawalObserver
            from table_withdrawal import TableWithdrawal
            TableWithdrawal(self.rt,self.session_motion,WithdrawalObserver(self,self.table,lambda:None),
                label='rotation-cycle-withdraw-'+str(self.serial))()
        self.rt.prepare_navigation_from_current()
        head=getattr(self,'identity_head_goal',(0.,0.))
        if getattr(self.policy,'template_family','')=='neck_bottle':
            index=getattr(self,'neck_view_retry',0)
            head=(0.,(-.15,-.30,-.50,0.)[index%4]);self.neck_view_retry=index+1
            self.record(dict(event='neck_view_retry',attempt=index+1,head_goal=head))
        self.rt.face_source(head)
        self.preextension_anchor_confirmed=False
        self.static_precontact_anchor=False
        self.identity_confirmed=False;self.pose_locked=False;self.baseline=None;self.evidence=None
        self.lift_done=False;self.closed=False;self.retreat_steps=[]
        self.record(dict(event='rotation_cycle_reobserve',base_reposition_allowed=True,total_limit=None))

    def capture_recovery_hand_hint(self):
        """Bounded optional RGB acquisition while stopped; no new motion gate."""
        self.recovery_hand_hint=None
        try:
            import time
            from hand_camera_ray import rotation,HAND_RASTER_TO_FRAME
            item=self.rt._acquire_observation('paired',timeout=1.,processing_reserve_ns=0,
                capture_after_ns=time.time_ns()-300_000_000)
            s=item['snapshot'];hand=s['streams']['hand'];depth=s['streams']['depth']
            if abs(hand['stamp_ns']-depth['stamp_ns'])>100_000_000:raise ValueError('hand_head_frames_not_close_in_time')
            if hand['frame']!='hand_camera_rgb_frame' or np.any(np.asarray(s['hand_d'])!=0):
                raise ValueError('hand_camera_profile_not_supported')
            R,t=self._tf(s);camera=s['base_from_hand']
            self.recovery_hand_hint=dict(rgb=item['hand'][:,:,::-1].copy(),
                camera_world=np.asarray(camera['translation'])@R.T+t,
                rotation_world=R@rotation(camera['quaternion']),
                raster_to_frame=HAND_RASTER_TO_FRAME,k=np.asarray(s['hand_k']).reshape(3,3),
                reference_rgb=self.colours.copy())
            path=self.rt.save(item,'recovery-hand-view')
            self.record(dict(event='recovery_hand_rgb_acquired',capture=str(path),
                hand_depth_available=False,advisory_only=True))
        except Exception as exc:
            self.recovery_hand_hint=None
            self.record(dict(event='recovery_hand_rgb_unavailable',reason=str(exc),
                geometry_replan_still_allowed=True))

    def rank_rotation_recovery(self,decision,failed):
        from .rotation_recovery import choose
        R,t=self._tf(self.source_snapshot)
        return choose(decision,failed,world_point=lambda p:np.asarray(p)@R.T+t,
                      hint=getattr(self,'recovery_hand_hint',None))

    def recover(self,action):
        if self.holding_state()!='empty':return
        if action['action']=='reobserve_then_correct_pose' and self.candidate is not None:
            # Check the NEW observed shape before another same-strategy close.
            self.pose_locked=False
            item,m=self._observe_tracking()
            from .templates import select_template
            scene=Scene(self.observed_contact_points,np.empty((0,3)),self._current_plane(item['snapshot']),self.last_stamp)
            policy,selection=select_template(self.target_name,scene,self.policy)
            if self.candidate['strategy']=='side' and policy.preferred_strategy=='overhead':
                self.record(dict(event='toppled_target_replan_before_correction',selection=selection))
                action=dict(action,action='withdraw_empty_reobserve_replan')
        if action['action']=='reobserve_then_correct_pose' and self.candidate is not None:
            item,m=self._observe_tracking()
            error=self.rt.contact_error(m['center_m'],item['snapshot'])
            if self.candidate['strategy']=='side':
                # Keep the fingers outside the object while correcting sideways.
                # At close range withdraw only enough to regain this clearance.
                if abs(error[1])>.006 and error[0]<.09:
                    self._base([float(error[0]-.10),0.,0.],'generic-clear-for-lateral-correction')
                    item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                if abs(error[1])>.006:
                    self._base([0.,float(error[1]),0.],'generic-correct-lateral-pose')
                item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                if abs(error[2])>.003:
                    goal=self.rt.state()['joints'].copy();goal[0]+=float(error[2])
                    self._arm(goal,'generic-correct-contact-height')
                    item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                if abs(error[0])>.003:
                    self._base([float(error[0]),0.,0.],'generic-correct-forward-contact')
            else:
                # Sideways correction is permitted while above the target only.
                if error[2]>-.04:
                    goal=self.rt.state()['joints'].copy();goal[0]+=.06
                    self._arm(goal,'generic-clear-above-for-correction')
                    item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                self._base([float(error[0]),float(error[1]),0.],'generic-correct-overhead-pose')
                item,m=self._observe_tracking();error=self.rt.contact_error(m['center_m'],item['snapshot'])
                goal=self.rt.state()['joints'].copy();goal[0]+=float(error[2])
                self._arm(goal,'generic-correct-overhead-contact-height')
            # Continue this same candidate after correction. Replanning here
            # used to move back to the standoff and undo the useful correction.
            item,m=self._observe_tracking()
            self._finish_grasp(item,m)
            return
        if action['open_hand']:
            goal=self.rt.state()['joints'].copy();goal[5]=.65
            self.rt.move_joints(goal,'generic-open-confirmed-empty',hand=True)
        # Reverse only this attempt's completed open-hand path. Fresh full
        # scene measurement precedes each reverse segment.
        for kind,start,end in reversed(self.retreat_steps):
            self._sample_for_path()
            if kind=='base':
                delta=(-np.asarray(start)).tolist();arm=self.rt.state()['joints']
                for f in np.linspace(0,1,max(2,int(np.linalg.norm(delta[:2])/.01)+1)):
                    self._check_open_path([arm],translation=np.r_[np.asarray(delta[:2])*f,0.])
                self.rt.base_move(delta,arm[:5],'generic-withdraw-empty')
            else:
                now=self.rt.state()['joints'];start[5]=now[5]
                self._check_open_path(np.linspace(now,start,max(2,int(np.max(abs(np.asarray(now)[:5]-np.asarray(start)[:5]))/.04)+1)))
                self.rt.move_joints(start,'generic-undo-empty-arm-step')
        self.retreat_steps=[]
        if action['action']=='change_observation_replan':
            if (not getattr(self,'identity_confirmed',False)
                    and not getattr(self,'view_withdrawn',False)
                    and abs(self.rt.state()['joints'][1])>1.):
                # If the process starts with an extended empty hand obscuring
                # the target, recover observation by withdrawing, not by
                # repeatedly tilting the head at the same blocked viewpoint.
                from can_destination_surface import measure
                from local_handle_geometry import plane_base
                self.latest_item=self._sample();s=self.latest_item['snapshot']
                plane=plane_base(s,measure(self.latest_item['depth'],s,self.table)['plane_map'])
                points,_,_=self._cloud(self.latest_item)
                support=points[abs(points[:,2]-points[:,:2]@plane[:2]-plane[2])<.008]
                if len(support)<30:raise AttemptFailure(Failure.OCCLUDED,'support_not_observed_for_empty_view_recovery')
                R,t=self._tf(s)
                # Workspace anchor only, never used as a target or grasp point.
                self.template=np.median(support,axis=0)[None,:]@R.T+t
                self.source_snapshot=s;self.plane=plane
                self._base([-.30,0.,0.],'generic-withdraw-to-clear-observation')
                self.retreat_steps=[];self.view_withdrawn=True
            # A new head view is useful even if no approach was attempted.
            tilt=-.45 if self.serial%2 else -.7
            self.rt.face_source((0.,tilt))
        self.closed=False;self.baseline=None;self.lift_done=False
        if action.get('failure') in ('empty_close','slipped','misaligned'):
            self.identity_confirmed=False;self.pose_locked=False
            self.record(dict(event='failed_grasp_requires_fresh_identity_and_shape',
                             reason=action.get('failure'),old_surface_template_reused=False))

    def stop(self):
        # Commands above are synchronous; never open or move the hand here.
        self.record(dict(event='generic_attempt_finished',holding=self.holding))
