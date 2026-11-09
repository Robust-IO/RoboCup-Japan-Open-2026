"""Image-time room check and coarse center consistency, not grasp permission."""
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parent/'scripts'))
from search_room_gate import RoomGate


def room_check(candidate,snapshot,environment,target_room):
    stamp=candidate['stamp_ns'];tf=snapshot['tracking_transforms']['map_from_base_at_depth']
    if candidate.get('source')!='rgbd' or candidate.get('frame_id')!='base_link':raise ValueError('invalid_candidate_frame')
    if stamp!=snapshot['streams']['depth']['stamp_ns'] or tf['stamp_ns']!=stamp:raise ValueError('map_transform_not_image_time')
    if tf['source_frame']!='base_link' or tf['target_frame']!='map':raise ValueError('wrong_map_transform')
    rooms={name:entry['region'] for name,entry in environment['rooms'].items()}
    result=RoomGate(rooms,target_room).classify(candidate['center_m'],tf['translation'],tf['quaternion'])
    return dict(result,room_confirmed=result['state']=='inside',layout=environment['internal_name'],
                grasp_center_verified=False,motion_authorized=False)


class CenterConsistency:
    def __init__(self):self.history=[];self.epoch=None
    def observe(self,candidate,room,epoch,fresh):
        result=dict(coarse_center_consistent=False,grasp_center_verified=False,motion_authorized=False)
        try:
            if not fresh or not room['room_confirmed']:raise ValueError('fresh_in_room_candidate_required')
            if not epoch:raise ValueError('missing_epoch')
            if self.epoch!=epoch:self.history=[];self.epoch=epoch
            stamp=candidate['stamp_ns'];point=room['position_map_m'];radius=candidate['radius_m']
            if not all(math.isfinite(v) for v in [*point,radius,candidate['fit_residual_p95_m']]):raise ValueError('nonfinite_geometry')
            if not .015<=radius<=.065 or not 0<=candidate['fit_residual_p95_m']<=.012:raise ValueError('bad_geometry')
            if self.history:
                gap=(stamp-self.history[-1][0])/1e9
                if gap<=0:raise ValueError('duplicate_or_reversed_stamp')
                if gap>1:self.history=[]
            if any(math.dist(point,p)>.01 or abs(radius-r)>.005 for _,p,r in self.history):raise ValueError('inconsistent_center_or_radius')
            self.history.append((stamp,list(point),radius));self.history=self.history[-3:]
            span=(stamp-self.history[0][0])/1e9
            result.update(coarse_center_consistent=len(self.history)==3 and span>=.2,
                          frames=len(self.history),span_s=span,
                          limitation='Repeatability is not absolute accuracy; contact-point and support-surface verification still required.')
        except (ValueError,KeyError) as exc:
            self.history=[];result['reason']=str(exc)
        return result
