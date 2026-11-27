"""Single-view adapter for saved diagnostic JSON. No ROS or robot commands.

Start only after the caller independently verifies arrival and settling.
Old logs lack whole-view depth health, so empty detections cannot certify absence.
"""
import math
from rgbd_localization import AUDIT_SHA256

class SearchVisionAdapter:
    def __init__(self,scan,view_id,arrival_stamp_ns):
        if scan.phase!='observe' or scan.request()['view_id']!=view_id or arrival_stamp_ns<=0:
            raise ValueError('view_not_ready')
        self.scan=scan;self.view_id=view_id;self.arrival=arrival_stamp_ns
        self.last_rgb=arrival_stamp_ns;self.last_depth=arrival_stamp_ns
        self.streak=[]

    def consume(self,row,now,*,sensor_now_ns):
        def reject(reason):
            self.streak=[]
            state=self.scan.observe(self.view_id,self.scan.target,0,now,False)
            return dict(state,adapter_reason=reason)
        try:
            if row.get('target')!=self.scan.target: return reject('wrong_target')
            if row.get('geometry_verified') is not True or row.get('audit_sha256')!=AUDIT_SHA256:
                return reject('unreviewed_geometry')
            rgb,depth=row['rgb_stamp_ns'],row['depth_stamp_ns']
            if not all(type(v) is int for v in [rgb,depth,sensor_now_ns]): return reject('invalid_timestamp')
            if rgb<=self.last_rgb or depth<=self.last_depth: return reject('old_or_duplicate_frame')
            self.last_rgb,self.last_depth=rgb,depth
            if abs(rgb-depth)>50_000_000 or not 0<=sensor_now_ns-min(rgb,depth)<=2_000_000_000:
                return reject('unsynchronised_or_stale')
            if row.get('tf_wait_status')!='ready' or row.get('tf_at_depth_stamp') is None:
                return reject('missing_tf')
            q=row['quality']; inference=row['inference']
            if inference.get('class_conflicts') or 'error' in inference: return reject('invalid_inference')
            detections=[d for d in inference['detections'] if d['name']==self.scan.target]
            if not detections:
                health=inference.get('view_health',{})
                if not isinstance(health,dict): return reject('no_target_with_unusable_view_data')
                if health.get('schema')=='handyman-view-health-v1':
                    if health.get('data_usable') is not True:
                        return reject('no_target_with_unusable_view_data')
                    return reject('no_target_data_usable_but_coverage_unverified')
                return reject('no_target_but_view_coverage_unverified')
            # Historical logs may omit health; an explicit failed or unknown
            # health report must never contribute to a positive streak.
            if 'view_health' in inference:
                health=inference['view_health']
                if not isinstance(health,dict) or health.get('schema')!='handyman-view-health-v1' or health.get('data_usable') is not True:
                    return reject('target_with_unusable_view_data')
            if len(detections)!=1 or q.get('stable') is not True: return reject('target_not_stable')
            if q.get('frame_id')!='odom' or q.get('target')!=self.scan.target or q.get('stamp_ns')!=depth:
                return reject('mismatched_localization')
            point=q['position_m']
            if len(point)!=3 or not all(math.isfinite(v) for v in point): return reject('invalid_position')
            room_check=getattr(self.scan,'room_check',None)
            if room_check is not None:
                room=room_check(point,depth)
                if room.get('state')!='inside':
                    return dict(reject('target_room_'+room.get('state','unverified')),room_membership=room)
            if any(math.dist(point,p)>.02 for p in self.streak): self.streak=[]
            self.streak.append(tuple(point)); self.streak=self.streak[-3:]
            state=self.scan.observe(self.view_id,self.scan.target,depth,now,True,
                stable=len(self.streak)==3,position=point)
            if state.get('state')=='found' and room_check is not None:
                self.scan.room_membership=room
            return dict(state,adapter_reason='fresh_stable_target' if len(self.streak)==3 else 'accumulating_new_view',
                        fresh_view_count=len(self.streak),
                        **({'room_membership':room} if room_check is not None else {}))
        except (KeyError,TypeError,ValueError,OverflowError): return reject('malformed_diagnostic')
