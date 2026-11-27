"""Current hand/held envelope against exported furniture bounds; no plane refit."""
import numpy as np

class WithdrawalObserver:
    def __init__(self,backend,table,context_provider,*,measure=None,map_point=None,rotation=None):
        # Keep the legacy keyword compatible; withdrawal no longer fits a plane.
        if map_point is None:
            from apple_transfer_auto_plan import map_point
        if rotation is None:
            from hand_camera_ray import rotation
        self.backend=backend;self.table=table;self.context_provider=context_provider
        self.map_point=map_point;self.rotation=rotation
        self.serial=0;self.last_stamp=None
        import uuid
        self.capture_id=uuid.uuid4().hex[:12]

    def __call__(self):
        item=self.backend._sample();s=item['snapshot'];stamp=s['streams']['depth']['stamp_ns']
        if self.last_stamp is not None and stamp<=self.last_stamp:raise ValueError('new_withdrawal_observation_required')
        palm=self.backend.rt.palm_pose(s);r=self.rotation(palm['quaternion'])
        opening=float(s['joints']['hand_motor_joint'])
        parts=self.backend.physical.sample(opening)['parts']
        local=[np.asarray(p['vertices_palm_m'],float) for p in parts if not p.get('is_trigger')]
        context=self.context_provider()
        if context is not None:
            if context.get('frame')!='hand_palm_link':raise ValueError('held_frame_mismatch')
            local.append(np.asarray(context['vertices_palm_m'],float))
        vertices=np.concatenate(local)
        base=vertices@r.T+np.asarray(palm['translation'])
        mapped=np.asarray([self.map_point(s,p) for p in base])
        # The furniture is static. Recompute the hand/held projection from fresh
        # TF, but do not require the support surface to remain visible.
        b=self.table['axis_aligned_bounds'];center=np.array([b['center']['x'],b['center']['y']])
        half=np.array([b['size']['x'],b['size']['y']])/2
        tf=s['tracking_transforms']['map_from_base_at_depth'];heading=self.rotation(tf['quaternion'])[:2,0]
        heading=heading/np.linalg.norm(heading)
        self.backend.rt.fresh_measurement(s);self.last_stamp=stamp;self.serial+=1
        result=dict(points_map=mapped.tolist(),table_center=center.tolist(),table_half=half.tolist(),
                    heading=heading.tolist(),depth_stamp_ns=stamp,includes_held_surface=context is not None,
                    furniture_bounds_basis="exported_axis_aligned_bounds",plane_redetection_required=False,
                    full_arm_sweep_verified=False)
        path=self.backend.rt.save(item,f'withdrawal-{self.table["id"]}-{self.capture_id}-{self.serial:03d}',result)
        result['evidence_ref']=str(path)
        return result
