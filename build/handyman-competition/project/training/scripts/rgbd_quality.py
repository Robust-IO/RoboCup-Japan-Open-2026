"""Fail-closed localization quality and temporal stability; no ROS or controls.

Coordinates represent visible surface samples, never an object centre/grasp pose.
Deployment must explicitly verify depth registration and optical frame convention.
"""
import math
from collections import deque


def transform_point(point, translation, quaternion):
    if len(point)!=3 or len(translation)!=3 or len(quaternion)!=4:
        raise ValueError('transform_shape')
    if not all(math.isfinite(v) for v in (*point,*translation,*quaternion)):
        raise ValueError('transform_nonfinite')
    norm=math.sqrt(sum(v*v for v in quaternion))
    if abs(norm-1)>1e-3:
        raise ValueError('quaternion_not_unit')
    x,y,z,w=(v/norm for v in quaternion)
    px,py,pz=point
    tx,ty,tz=2*(y*pz-z*py),2*(z*px-x*pz),2*(x*py-y*px)
    return (px+w*tx+y*tz-z*ty+translation[0],
            py+w*ty+z*tx-x*tz+translation[1],
            pz+w*tz+x*ty-y*tx+translation[2])


class StabilityGate:
    """Consecutive unique captures of one unambiguous target in a fixed frame.

    All timestamps use the same ROS clock. Invalid observations reset history;
    callers must also call reject on missing detections and stream timeouts.
    """
    def __init__(self, count=3, max_gap_s=5., max_age_s=2., tolerance_m=.02):
        if count<2 or min(max_gap_s,max_age_s,tolerance_m)<=0:
            raise ValueError('invalid_gate_config')
        self.history=deque(maxlen=count)
        self.count=count
        self.max_gap_s=max_gap_s
        self.max_age_s=max_age_s
        self.tolerance_m=tolerance_m

    def reject(self, reason):
        self.history.clear()
        return {'status':'rejected','reason':reason,'stable':False}

    def update(self, observation, now_ns):
        o=observation
        try:
            if not o['alignment_verified'] or not o['optical_frame_verified']:
                return self.reject('unverified_geometry')
            if o['instances']!=1 or o['class_conflict']:
                return self.reject('ambiguous_target')
            if not math.isfinite(o['confidence']) or not .5<=o['confidence']<=1:
                return self.reject('low_confidence')
            rgb,depth=o['rgb_stamp_ns'],o['depth_stamp_ns']
            if min(rgb,depth)<=0 or abs(rgb-depth)>50_000_000:
                return self.reject('unsynchronised')
            if min(now_ns-rgb,now_ns-depth)<0 or max(now_ns-rgb,now_ns-depth)>self.max_age_s*1e9:
                return self.reject('stale_or_future')
            if o['tf_stamp_ns']!=depth:
                return self.reject('tf_timestamp_mismatch')
            if o['core_pixels']<50 or not .8<=o['valid_pixels']/o['core_pixels']<=1:
                return self.reject('insufficient_depth')
            lo,mid,hi=o['depth_p05_median_p95']
            if not all(math.isfinite(v) for v in (lo,mid,hi)) or not .15<=lo<=mid<=hi<=5. or hi-lo>.08:
                return self.reject('depth_spread_or_range')
            p=o['position_m']
            if len(p)!=3 or not all(math.isfinite(v) for v in p):
                return self.reject('invalid_position')
            if not o['target'] or not o['frame_id']:
                return self.reject('missing_identity')
            if self.history:
                prev=self.history[-1]
                if rgb<=prev['rgb_stamp_ns'] or depth<=prev['depth_stamp_ns']:
                    return self.reject('duplicate_or_reversed_time')
                if (o['target'],o['frame_id'])!=(prev['target'],prev['frame_id']) or (depth-prev['depth_stamp_ns'])/1e9>self.max_gap_s:
                    self.history.clear()
                elif any(math.dist(p,item['position_m'])>self.tolerance_m for item in self.history):
                    self.history.clear()
            self.history.append(dict(o,position_m=tuple(p)))
            stable=len(self.history)==self.count
            return {'status':'stable' if stable else 'accumulating','stable':stable,'count':len(self.history),
                    'frame_id':o['frame_id'],'target':o['target'],'position_m':list(p),'stamp_ns':depth}
        except (KeyError,TypeError,ValueError,ZeroDivisionError,OverflowError):
            return self.reject('malformed_observation')
