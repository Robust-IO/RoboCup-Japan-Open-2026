"""Adapt verified RGB-D diagnostics to a surface planning candidate, no motion.

Caller binds task identity, room result and image-time TF to this exact capture.
This does not convert a surface sample into an object centre or a grasp pose.
"""
import math
from grasp_target_gate import TargetEvidence, check_target
from rgbd_quality import transform_point
from rgbd_localization import AUDIT_SHA256


def adapt(row, *, task_id, expected_task_id, target, now_ns, room,
          base_from_odom):
    try:
        if not task_id or task_id != expected_task_id:
            raise ValueError('task_mismatch')
        if row.get('geometry_verified') is not True or row.get('audit_sha256') != AUDIT_SHA256:
            raise ValueError('unverified_geometry')
        rgb, depth = row['rgb_stamp_ns'], row['depth_stamp_ns']
        if any(type(v) is not int or v <= 0 for v in (rgb, depth, now_ns)):
            raise ValueError('invalid_timestamp')
        if abs(rgb-depth)>50_000_000 or not 0<=now_ns-rgb<=500_000_000:
            raise ValueError('unsynchronised_or_stale_rgb')
        q=row['quality']; inference=row['inference']
        if row.get('target') != target or q.get('target') != target or q.get('stable') is not True:
            raise ValueError('target_not_stable')
        if q.get('frame_id')!='odom' or q.get('stamp_ns')!=depth or row.get('tf_wait_status')!='ready':
            raise ValueError('localization_mismatch')
        if inference.get('class_conflicts') or 'error' in inference:
            raise ValueError('ambiguous_inference')
        ds=[d for d in inference['detections'] if d['name']==target]
        if len(ds)!=1: raise ValueError('ambiguous_target')
        d=ds[0]
        if not math.isfinite(d['confidence']) or not .5<=d['confidence']<=1:
            raise ValueError('low_confidence')
        lo,mid,hi=d['depth_p05_median_p95']
        if not all(math.isfinite(v) for v in (lo,mid,hi)) or not 0<lo<=mid<=hi or hi-lo>.08:
            raise ValueError('invalid_depth_distribution')
        if d['core_pixels']<50 or not .8<=d['valid_pixels']/d['core_pixels']<=1:
            raise ValueError('insufficient_depth')
        if room.get('stamp_ns')!=depth or room.get('state')!='inside':
            raise ValueError('room_evidence_mismatch')
        tf=base_from_odom
        if (tf['source_frame'],tf['target_frame'],tf['stamp_ns'])!=('odom','base_link',depth):
            raise ValueError('transform_mismatch')
        point=transform_point(q['position_m'],tf['translation'],tf['quaternion'])
        evidence=TargetEvidence(task_id,target,'base_link',depth/1e9,point,mid,True,True)
        ok,reason=check_target(evidence,task_id=expected_task_id,class_name=target,now=now_ns/1e9)
        return dict(eligible_for_planning=ok,reason=reason,evidence=evidence if ok else None,
                    position_semantics='visible_surface_not_grasp_pose',motion_authorized=False)
    except (KeyError,TypeError,ValueError,OverflowError,ZeroDivisionError) as error:
        return dict(eligible_for_planning=False,reason=str(error),evidence=None,motion_authorized=False)
