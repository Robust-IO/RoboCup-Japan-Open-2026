"""Point-local evidence, not room coverage. Consumes already-validated adapter results."""
import math


class ViewEvidence:
    def __init__(self):
        self.reset()

    def reset(self):
        self.arrival_stamp=None;self.negatives=[];self.last_now=None;self.excluded=[]

    def arrived(self,stamp):
        self.reset()
        if type(stamp) is int and stamp>0:self.arrival_stamp=stamp

    def observe(self,row,result,now):
        # The adapter ticks the scan before processing a frame. At the view
        # deadline this frame belongs outside the observation window: preserve
        # existing evidence, do not count the late frame or erase prior samples.
        # Completed evidence is historical within this stationary view epoch.
        # Invalid data or a new arrival still clears it.
        if (result.get('state')=='incomplete' and
                result.get('failures')==['0:observation_timeout']):
            return
        stamp=row.get('depth_stamp_ns')
        membership=result.get('room_membership',{})
        excluded=(result.get('adapter_reason')=='target_room_outside_target_room' and
                  membership.get('state')=='outside_target_room' and
                  row.get('inference',{}).get('view_health',{}).get('data_usable') is True)
        eligible=(self.arrival_stamp is not None and result.get('state')=='observe' and
            (result.get('adapter_reason')=='no_target_data_usable_but_coverage_unverified' or excluded) and
            type(stamp) is int and stamp>self.arrival_stamp and math.isfinite(now))
        if not eligible:
            self.negatives=[];self.excluded=[];self.last_now=None;return
        if self.last_now is not None and (now<=self.last_now or stamp<=self.negatives[-1]):
            self.negatives=[];self.excluded=[];self.last_now=None;return
        self.negatives.append(stamp);self.negatives=self.negatives[-32:];self.last_now=now
        if excluded:self.excluded.append(stamp)
        self.excluded=[s for s in self.excluded if s in self.negatives]

    def snapshot(self,now):
        fresh=(self.last_now is not None and math.isfinite(now) and 0<=now-self.last_now<=.5)
        return dict(schema='handyman-view-evidence-v1',arrival_verified=self.arrival_stamp is not None,
                    arrival_stamp_ns=self.arrival_stamp,negative_stamps=[s for s in self.negatives if s not in self.excluded],
                    continuation_stamps=list(self.negatives),outside_room_stamps=list(self.excluded),
                    continuation_scope='view_only_not_room_absence',
                    current_data_fresh=fresh,
                    view_data_usable=bool(math.isfinite(now) and self.last_now is not None and
                                          now>=self.last_now and len(self.negatives)>=3),coverage_verified=False)
