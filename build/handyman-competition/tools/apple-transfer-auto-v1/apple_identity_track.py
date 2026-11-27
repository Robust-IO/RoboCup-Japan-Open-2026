"""Separate confirmed identity from fresh RGBD association. No motion authority.

Input observations must contain independently measured, odom-frame RGBD centers.
Predicted/retimestamped old coordinates cannot renew this track. Apple-specific.
"""
import math


class AppleIdentityTrack:
    def __init__(self, task_id, epoch):
        if not task_id or not epoch:
            raise ValueError('task_and_epoch_required')
        self.task_id, self.epoch = task_id, epoch
        self.previous = None
        self.seed_frames = 0
        self.confirmed = False

    def reset(self):
        self.previous = None
        self.seed_frames = 0
        self.confirmed = False

    def update(self, observations, *, stamp_ns, now_ns, task_id, epoch,
               room_confirmed, contact_started=False):
        def reject(reason):
            self.reset()
            return dict(status='reobserve', reason=reason, identity_confirmed=False,
                        motion_authorized=False, grasp_center_verified=False)
        if task_id != self.task_id or epoch != self.epoch:
            return reject('task_or_epoch_changed')
        if type(stamp_ns) is not int or type(now_ns) is not int or stamp_ns <= 0:
            return reject('invalid_timestamp')
        if not 0 <= (now_ns-stamp_ns)/1e9 <= .4:
            return reject('stale_observation')
        if contact_started or room_confirmed is not True:
            return reject('contact_or_room_unverified')
        reconfirm_after_gap = False
        if self.previous:
            gap=(stamp_ns-self.previous['stamp_ns'])/1e9
            if gap <= 0:
                return reject('observation_gap_or_reversed_stamp')
            # Missing frames expire a motion lease, not the object's class.
            # Re-entry still needs a NEW strong classification plus the same
            # measured spatial/radius association. Weak continuity cannot span
            # a gap, and old/predicted frames fail the freshness checks above.
            reconfirm_after_gap = gap > .8
        valid=[]
        for item in observations:
            point=item.get('point_odom_m', [])
            numbers=list(point)+[item.get('radius_m',float('nan')),
                                 item.get('confidence',float('nan')),
                                 item.get('fit_residual_p95_m',float('nan'))]
            if len(point)!=3 or not all(isinstance(x,(float,int)) and math.isfinite(x) for x in numbers):
                return reject('invalid_geometry')
            if (item.get('measurement')!='fresh_rgbd' or item.get('stamp_ns')!=stamp_ns
                    or item.get('class_name')!='apple' or item.get('red_appearance') is not True
                    or item.get('clipped') is not False or item.get('depth_valid') is not True
                    or not .015<=item['radius_m']<=.065
                    or not 0<=item['fit_residual_p95_m']<=.012
                    or not 0<=item['confidence']<=1):
                continue
            if self.previous:
                if math.dist(point,self.previous['point_odom_m'])>.025:continue
                if abs(item['radius_m']-self.previous['radius_m'])>.008:continue
            # Identity is appearance + measured RGBD association, not proof of
            # table contact. The source plan separately requires measured
            # support before preparation. A partly occluded sphere-bottom
            # estimate must not erase an otherwise valid current identity.
            valid.append(item)
        # Ambiguity is checked before confidence. A low-score nearby twin is not ignored.
        if len(valid)!=1:
            return reject('missing_or_ambiguous_association')
        selected=valid[0]
        if reconfirm_after_gap and selected['confidence']<.8:
            return reject('identity_reconfirmation_required_after_gap')
        if not self.confirmed and selected['confidence']<.8:
            return reject('identity_seed_below_threshold')
        if not self.confirmed:
            self.seed_frames+=1
            self.confirmed=self.seed_frames>=3
        self.previous=dict(selected,point_odom_m=list(selected['point_odom_m']))
        return dict(status='identity_tracked' if self.confirmed else 'identity_seeding',
                    identity_confirmed=self.confirmed, measurement=selected,
                    identity_basis='three_high_confidence_frames_then_rgbd_association_with_strong_reconfirmation_after_gap',
                    gap_reconfirmed_with_current_classification=reconfirm_after_gap,
                    motion_authorized=False, grasp_center_verified=False)
