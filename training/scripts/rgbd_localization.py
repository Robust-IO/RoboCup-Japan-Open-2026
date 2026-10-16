"""Audited-profile diagnostic localization. Never grants motion authority."""
import hashlib
from pathlib import Path
from rgbd_quality import StabilityGate, transform_point

AUDIT_SHA256='595f1345ed1d656d48405fd56270d0d60d7837e48072c11ae86b0c044a7cb3da'
K=[554.,0.,320.,0.,554.,240.,0.,0.,1.]

def load_audit(path):
    if path is None: return False
    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=AUDIT_SHA256:
        raise ValueError('Camera audit differs from reviewed report; review required')
    return True

def camera_matches(rgb,depth,ci,di):
    return (rgb.width,rgb.height,depth.width,depth.height,ci.width,ci.height,di.width,di.height)==(640,480)*4 and (
        rgb.header.frame_id==ci.header.frame_id=='head_rgbd_sensor_rgb_frame' and
        depth.header.frame_id==di.header.frame_id=='head_rgbd_sensor_depth_frame' and
        depth.encoding=='16UC1' and not depth.is_bigendian and
        list(ci.k)==list(di.k)==K and not any(ci.d) and not any(di.d))

class DiagnosticLocalization:
    def __init__(self,audited=False):
        self.audited=audited
        self.gate=StabilityGate(max_gap_s=2.)

    def reject(self,reason):
        return self.gate.reject(reason)

    def evaluate(self,result,target,rgb_ns,depth_ns,now_ns,tf,tf_status):
        try:
            if not self.audited: return self.reject('unverified_geometry')
            if 'error' in result: return self.reject('inference_error')
            selected=[d for d in result['detections'] if d['name']==target]
            if not selected: return self.reject('target_not_in_current_frame')
            if len(selected)!=1 or result.get('class_conflicts'): return self.reject('ambiguous_target')
            if tf_status!='ready' or tf is None: return self.reject('missing_or_expired_tf')
            d=selected[0]
            if not .4<d['depth_p05_median_p95'][0]<=d['depth_p05_median_p95'][2]<8:
                return self.reject('outside_audited_depth_range')
            position=transform_point(d['optical_surface_median_m'],tf['translation'],tf['rotation_xyzw'])
            return self.gate.update(dict(alignment_verified=True,optical_frame_verified=True,
                instances=1,class_conflict=False,confidence=d['confidence'],rgb_stamp_ns=rgb_ns,
                depth_stamp_ns=depth_ns,tf_stamp_ns=depth_ns,core_pixels=d['core_pixels'],
                valid_pixels=d['valid_pixels'],depth_p05_median_p95=d['depth_p05_median_p95'],
                position_m=position,frame_id='odom',target=target),now_ns)
        except (KeyError,TypeError,ValueError,OverflowError):
            return self.reject('malformed_localization_result')
