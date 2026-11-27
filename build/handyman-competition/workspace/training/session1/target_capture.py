"""Acquire independent post-release target surface through the existing worker.

Backend._sample/_cloud provide synchronized RGBD and robot-surface removal.
No previous target template or held geometry is used in instance segmentation.
"""
import json
from pathlib import Path
import numpy as np

class TargetCapture:
    def __init__(self,backend,worker,destination,*,target='dog_doll',confidence=.4,
                 measure=None,map_point=None,component=None,expected_height_m=None):
        if measure is None:
            from can_destination_surface import measure
        if map_point is None:
            from apple_transfer_auto_plan import map_point
        if component is None:
            from grasp_runtime.autograsp.tracking import dominant_component as component
        self.backend=backend;self.worker=worker;self.destination=destination
        self.expected_height_m=expected_height_m
        self.target=target;self.confidence=confidence;self.measure=measure
        self.map_point=map_point;self.component=component;self.last_stamp=None;self.serial=0

    def capture(self,*,release_stamp_ns):
        item=self.backend._sample();snapshot=item['snapshot']
        stamp=snapshot['streams']['depth']['stamp_ns']
        if stamp<=release_stamp_ns or (self.last_stamp is not None and stamp<=self.last_stamp):
            raise ValueError('new_post_release_depth_required')
        self.last_stamp=stamp;self.serial+=1
        path=Path(self.backend.rt.save(item,f'placed-target-{self.serial:03d}'))
        detection=self.worker.infer(path,pump=lambda:self.backend.rt.pump(0))
        (path/'detections.json').write_text(json.dumps(detection,ensure_ascii=False))
        plane=self.measure(item['depth'],snapshot,self.destination)
        if plane['destination_id']!=self.destination['id'] or plane['source_depth_stamp_ns']!=stamp:
            raise ValueError('same_frame_target_table_required')
        points,_,indices=self.backend._cloud(item)
        origin=np.asarray(self.map_point(snapshot,np.zeros(3)))
        basis=np.stack([np.asarray(self.map_point(snapshot,axis))-origin for axis in np.eye(3)],axis=1)
        mapped=points@basis.T+origin
        if not len(mapped):raise ValueError('no_valid_target_depth')
        v,u=np.unravel_index(indices,item['depth'].shape)
        height=mapped[:,2]-np.c_[mapped[:,:2],np.ones(len(mapped))]@np.asarray(plane['plane_map'])
        bounds=self.destination['axis_aligned_bounds'];c=bounds['center'];size=bounds['size']
        in_table=(abs(mapped[:,0]-c['x'])<size['x']/2)&(abs(mapped[:,1]-c['y'])<size['y']/2)
        candidates=[]
        for d in detection['detections']:
            # Post-release identity corroboration: grasp already established
            # identity. Geometry and temporal stability still verify placement.
            if d['name']!=self.target or not d['confidence']>self.confidence:continue
            x1,y1,x2,y2=d['box']
            selected=np.flatnonzero((u>=x1)&(u<x2)&(v>=y1)&(v<y2)&in_table&(height>.006)&(height<.4))
            if len(selected)<30:continue
            _,keep=np.unique(np.floor(mapped[selected]/.003).astype(int),axis=0,return_index=True)
            selected=selected[keep]
            try:selected=selected[self.component(mapped[selected])]
            except ValueError:continue
            if self.expected_height_m is not None:
                # Broad geometric consistency from this task's measured object,
                # not a class-specific size or an exact expected placement pose.
                minimum=max(.006,min(.04,.35*self.expected_height_m))
                if float(np.quantile(height[selected],.95))<minimum:continue
            # Duplicate detector boxes covering the same component do not count
            # as multiple objects. Distinct same-class objects remain ambiguous.
            if any(len(set(selected)&set(old))>=.6*min(len(selected),len(old)) for old in candidates):continue
            candidates.append(selected)
        if len(candidates)!=1:raise ValueError('placed_target_not_uniquely_observed_on_destination')
        selected=candidates[0];surface=mapped[selected]
        row=dict(target=self.target,destination_id=self.destination['id'],depth_stamp_ns=stamp,
                 geometry_source='independent_target_RGBD',points_map=surface.tolist(),plane=plane,
                 bottom_observed=bool(np.min(height[selected])<=.03),
                 bottom_evidence='visible_target_surface_in_table_contact_height_band',
                 evidence_ref=str(path),identity_source='current_detector_and_connected_RGBD')
        (path/'target-surface.json').write_text(json.dumps(row,ensure_ascii=False))
        # Inference can take longer than the motion freshness window. This is a
        # historical post-release observation, not a motion publication permit.
        return row

    def verify(self,plan,*,release_stamp_ns,required_frames=3,maximum_samples=9):
        """Collect distinct observations and feed the Session1 verification stage.

        release_stamp_ns must use the sensor clock, not wall-clock time. Caller
        must establish release acknowledgement in that clock and current session.
        """
        from session_table_support import support_from_plan
        import task_outcome_policy
        from placement_verification import verify_placement
        rows=[];errors=[]
        for _ in range(maximum_samples):
            try:
                row=self.capture(release_stamp_ns=release_stamp_ns)
                if rows and row['depth_stamp_ns']-rows[-1]['depth_stamp_ns']<150_000_000:
                    self.backend.rt.pump(.1);continue
                rows.append(row)
                if len(rows)>=required_frames:break
            except ValueError as exc:
                errors.append(str(exc));self.backend.rt.pump(.1)
        if len(rows)<required_frames:
            return dict(status='incomplete',placement_verified=False,reason='insufficient_independent_target_observations',observation_errors=errors,observed_frames=len(rows))
        result=verify_placement(rows,plan,support_from_plan(plan),task_outcome_policy,
                                target=self.target,release_stamp_ns=release_stamp_ns,
                                joints=self.backend.rt.state()['joints'],required_frames=required_frames)
        result['observation_errors']=errors
        return result
