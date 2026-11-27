"""HSR container actions reusing the measured grasp posture and finite motions."""
import json
from itertools import count
from task_recovery import check_cancel,motion_cancel,retry_read
import math
import numpy as np
from measured_release_posture import MeasuredReleaseCore
from transfer_actions import measured_release_shape
from grasp_runtime.autograsp.kinematics import CONTACT
from container_floor_capture import ContainerFloorCapture
from container_transfer import release_ready,ContainerPolicy

class ContainerLiveAdapter:
    def __init__(self,stages,context,*,core=None,map_point=None,rotation=None):
        if core is None:import table_transfer_core as core
        if map_point is None:from apple_transfer_auto_plan import map_point
        if rotation is None:from hand_camera_ray import rotation
        self.stages=stages;self.rt=stages.runtime;self.backend=stages.grasp_backend
        self.context=context;self.map_point=map_point;self.rotation=rotation
        self.core=MeasuredReleaseCore(core,context['joints_at_export'])
        self.shape=measured_release_shape(context,self.core.rotation)
        self.opening=dict(stages.config['container_opening'])
        self.released=False;self.plan=None;self.staging=None
        self.floor=ContainerFloorCapture(self.backend,self.backend.worker,
            stages.config['container_destination'],target=stages.target,map_point=map_point,opening=self.opening)

    def prepare_low_bin_view(self):
        """Observe outside the mouth at a lower height; never release here."""
        height=float(self.opening['rim_height_m'])+self.bottom_below_palm_m()+.04
        height=max(self.core.offset,min(self.core.offset+.69,height))
        snapshot=self._snapshot()
        predicted=self.core.predict(height,self.core.rotation@CONTACT)
        mapped=np.asarray(self.map_point(snapshot,predicted))
        polygon=np.asarray(self.opening['polygon_map_m'])
        gap=float(np.linalg.norm(np.maximum(np.maximum(polygon.min(0)-mapped[:2],mapped[:2]-polygon.max(0)),0)))
        if gap<.12:
            self.backend.record(dict(event='container_low_view_posture_skipped',
                reason='predicted_hand_already_near_mouth',mouth_gap_m=gap))
            return
        opening=float(self.rt.state()['joints'][5])
        self.core.prepare_release(self.rt,height,opening)
        self.backend.record(dict(event='container_low_view_posture_ready',
            observation_palm_height_m=height,requested_bottom_above_rim_m=.04,
            mouth_gap_m=gap,release_sent=False))

    def observe_opening(self,destination):
        if self.opening['destination_id']!=destination:raise ValueError('container_geometry_destination_mismatch')
        self.rt.sample(profile='state')
        if self.stages.config.get('session_profile',{}).get('session_number') in (2,4):
            from container_rim_calibration import calibrate
            from container_route_heading import face_container_route
            face_container_route(self.stages,self.opening)
            self.prepare_low_bin_view()
            last=None
            for head in ((0.,-.3),(0.,-.5),(0.,-.15)):
                check_cancel(motion_cancel(self.stages.motion))
                self.rt.face_source(head)
                item=self.backend._sample();snapshot=item['snapshot']
                self.rt.fresh_measurement(snapshot)
                evidence=str(self.rt.save(item,'container-rim-'+str(snapshot['streams']['depth']['stamp_ns'])))
                # Save measured joint/TF geometry with the RGBD view for comparison.
                palm=self.rt.palm_pose(snapshot)
                hand=self.map_point(snapshot,np.asarray(palm['translation'])+
                    self.rotation(palm['quaternion'])@CONTACT)
                self.backend.record(dict(event='container_hand_mouth_observation',
                    evidence_ref=evidence,gripper_center_map_m=list(map(float,hand)),
                    configured_mouth_center_map_m=np.asarray(self.opening['polygon_map_m']).mean(0).tolist(),
                    hand_position_source='joint_kinematics_and_depth_timestamp_tf',
                    visual_hand_segmentation_verified=False))
                points,_,_=self.backend._cloud(item)
                tf=snapshot['tracking_transforms']['map_from_base_at_depth']
                mapped=points@self.rotation(tf['quaternion']).T+np.asarray(tf['translation'])
                try:
                    opening,details=calibrate(mapped,self.opening)
                except ValueError as exc:
                    last=exc;self.backend.record(dict(event='container_rim_observation_retry',reason=str(exc),evidence_ref=evidence));continue
                self.opening.clear();self.opening.update(opening)
                self.backend.record(dict(event='container_rim_calibrated',evidence_ref=evidence,**details))
                break
            else:
                # The burnable bin has a sloping mouth. A horizontal-rim fit is
                # optional refinement, not evidence that configured geometry is invalid.
                # Retain the configured opening and all existing release checks.
                self.backend.record(dict(event='container_rim_calibration_fallback',
                    reason=str(last), destination=destination,
                    geometry_source=self.opening.get('source','configured'),
                    current_rim_verified=False, task_aborted=False,
                    release_geometry_checks_retained=True))
        return self.opening

    def bottom_below_palm_m(self):return self.shape['bottom_below_palm_m']

    def _snapshot(self):
        def read():
            item=self.backend._sample();self.rt.fresh_measurement(item['snapshot']);return item['snapshot']
        return retry_read(read,cancel=motion_cancel(self.stages.motion),record=self.backend.record)

    def current_gripper_and_bottom(self):
        snapshot=self._snapshot();palm=self.rt.palm_pose(snapshot);r=self.rotation(palm['quaternion'])
        center=self.map_point(snapshot,np.asarray(palm['translation'])+r@CONTACT)
        points=np.asarray(self.context['vertices_palm_m'])@r.T+np.asarray(palm['translation'])
        ref=self.context.get('source_support_reference')
        if ref:points=np.vstack([points,np.asarray(ref['point_palm_m'])@r.T+np.asarray(palm['translation'])])
        mapped=np.asarray([self.map_point(snapshot,p) for p in points])
        self.object_projection_map_m=mapped[:,:2]
        bottom=float(mapped[:,2].min())
        return center,bottom

    def release_margin(self):
        value=float(getattr(self.stages,'config',{}).get('container_release_inset_m',.03))
        if not math.isfinite(value) or not 0<=value<=.2:raise ValueError('invalid_container_release_inset')
        return value

    def release_geometry_ready(self,center,bottom):
        from container_geometry import projection_inside
        if not release_ready(center,bottom,self.opening):return False
        # Only the gripper center must fit the inset mouth; held-object XY projection is disabled.
        points=np.asarray(center,dtype=float)[:2].reshape(1,2)
        if not projection_inside(points,self.opening['polygon_map_m'],margin=self.release_margin()):return False
        plan=getattr(self,'plan',None)
        if plan is not None and plan.get('placement_selection')=='maximum_rim_clearance':
            return bool(np.linalg.norm(np.asarray(center[:2])-plan['gripper_xy_map_m'])<=.01)
        return True

    def approach(self,plan):
        from container_route_heading import face_container_route
        face_container_route(self.stages,self.opening)
        self.plan=plan;self.rt.sample(profile='state');state=self.rt.state()
        self.staging=list(state['poses']['map'])
        self.core.prepare_release(self.rt,plan['palm_height_m'],state['joints'][5])
        # Reach the best-clearance target, not merely the first valid edge point.
        for index in count():
            check_cancel(motion_cancel(self.stages.motion))
            center,bottom=self.current_gripper_and_bottom()
            from container_geometry import fit_release_xy
            offsets=np.zeros((1,2))  # Plan for gripper center only.
            plan['gripper_xy_map_m']=fit_release_xy(self.opening,offsets,
                plan['gripper_xy_map_m'],margin=self.release_margin())
            plan['container_release_inset_m']=self.release_margin()
            plan['placement_selection']='maximum_rim_clearance'
            self.backend.record(dict(event='container_release_centering',goal=plan['gripper_xy_map_m'],
                current=list(center[:2]),inset_m=self.release_margin(),position_tolerance_m=.01,
                approach_reduction_applied=False))
            if self.release_geometry_ready(center,bottom):return
            snapshot=self._snapshot();r=self.rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
            delta=r.T@np.r_[np.asarray(plan['gripper_xy_map_m'])-center[:2],0.]
            state=self.rt.state()
            if np.linalg.norm(delta[:2])<.001:
                goal=list(state['joints']);goal[0]+=self.opening['rim_height_m']+ContainerPolicy().release_bottom_above_rim_m-bottom
                if not 0<=goal[0]<=.69:raise ValueError('container_release_outside_arm_reach')
                self.rt.move_joints(goal,'container-correct-height')
            else:
                self.stages.motion.stage='approach_placement'
                self.stages.motion.adjust(f'container-approach-{index}',state['poses']['map'],[*map(float,delta[:2]),0.],state['joints'][:5],state['joints'][5],single_placement_approach=True)

    def release_once(self):
        if self.released:raise ValueError('container_release_already_sent')
        def check():
            center,bottom=self.current_gripper_and_bottom()
            if not self.release_geometry_ready(center,bottom):raise ValueError('gripper_center_or_height_outside_container')
            return dict(ready=True,gripper_center_map_m=list(center),bottom_height_m=bottom,visual_hold_required=False)
        self.core.release_open(self.rt,check);self.released=True
        snapshot=self._snapshot();stamp=snapshot['streams']['depth']['stamp_ns']
        record=dict(release_sent=True,release_stamp_ns=stamp,destination=self.stages.destination_id,
                    criterion='centered_gripper_inside_inset_opening',
                    held_projection_check_enabled=False,
                    placement_selection='maximum_rim_clearance',
                    container_release_inset_m=self.release_margin(),placement_verified=False)
        (self.stages.output/'container-release.json').write_text(json.dumps(record,indent=2))
        return stamp

    def withdraw_empty(self):
        self.core.raise_after_release(self.rt)
        snapshot=self._snapshot();state=self.rt.state()
        r=self.rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
        delta=r.T@np.r_[np.asarray(self.staging[:2])-state['poses']['map'][:2],0.]
        if np.linalg.norm(delta[:2])>.005:
            self.stages.motion.stage='withdraw'
            self.stages.motion.adjust('container-return-to-staging',state['poses']['map'],[*map(float,delta[:2]),0.],state['joints'][:5],state['joints'][5])
        self.core.fold_after_withdrawal(self.rt)
        goal=list(self.rt.state()['joints']);goal[2]=goal[4]=0.
        self.rt.move_joints(goal,'container-empty-navigation-posture')

    def capture_external_floor(self):return self.floor()

    def recover_floor_target(self,detections):
        # Explicitly report the missing recovery capability rather than reopening
        # at the bin or claiming a grasp without a physical lift verification.
        self.backend.record(dict(event='container_floor_recovery_needed',detections=detections))
        callback=getattr(self.stages,'recover_floor_target',None)
        if callback is None:
            from container_recovery import recover
            context=recover(self.stages,detections)
        else:context=callback(detections)
        if not context:return False
        self.__init__(self.stages,context)
        return True
