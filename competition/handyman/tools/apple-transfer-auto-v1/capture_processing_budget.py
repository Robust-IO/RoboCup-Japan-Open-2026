"""Measured scheduling hints, NEVER a replacement for final freshness checks."""
from collections import deque
import math
from synced_capture_selection import select_stamps


class ProcessingBudget:
    def __init__(self):
        self.costs = {}
        self.wait_started = None

    def observe(self, mode, seconds):
        if not isinstance(mode, str) or not mode or type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 0:
            raise ValueError('invalid_measured_processing_cost')
        self.costs.setdefault(mode, deque(maxlen=16)).append(seconds)

    def reserve_ns(self, mode):
        values = sorted(self.costs.get(mode, ()))
        if not values:
            return 0  # Bootstrap once from genuinely current data, not a guessed200ms.
        estimate = values[max(0, math.ceil(.9*len(values))-1)] + .005
        return min(399_000_000, math.ceil(min(.399,estimate)*1e9))

    def select(self, history, now_ns, last=None, *, ready, joint_stamp_ns, mode,
               monotonic_s=None):
        if monotonic_s is not None and (type(monotonic_s) not in (int,float)
                or not math.isfinite(monotonic_s) or monotonic_s<0):
            raise ValueError('invalid_scheduling_clock')
        estimate = self.reserve_ns(mode)
        # Live scheduling also leaves room for publishing/consumer file reads.
        # This is ONLY a preference, not a new data-validity threshold.
        reserve = min(399_000_000,estimate+30_000_000) if estimate and monotonic_s is not None else estimate
        kwargs = dict(ready=ready, joint_stamp_ns=joint_stamp_ns)
        selected = select_stamps(history, now_ns, last, processing_reserve_ns=reserve, **kwargs)
        fallback = selected is None and reserve > 0
        deferred=False
        if fallback and monotonic_s is not None:
            if self.wait_started is None:self.wait_started=monotonic_s
            deferred=monotonic_s-self.wait_started<.25
        if fallback and not deferred:
            # Still the ORIGINAL <=400ms selection/sync/TF checks. A forecast
            # must not permanently veto new data that might finish in time.
            selected = select_stamps(history, now_ns, last, processing_reserve_ns=0, **kwargs)
        if selected is not None:self.wait_started=None
        decision = dict(mode=mode, estimated_processing_reserve_ns=estimate,
            applied_processing_reserve_ns=0 if fallback and not deferred else reserve,
            publication_consumer_scheduling_headroom_ns=reserve-estimate,
            forecast_fallback_used=fallback and not deferred,bounded_new_feedback_wait=deferred,
            maximum_feedback_wait_s=.25,bootstrap=not bool(self.costs.get(mode)),
            final_original_stamp_freshness_required=True)
        return selected, decision


def result_timing(snapshot, now_ns, elapsed_s):
    """Check ORIGINAL captured joints AND both image stamps after real processing."""
    stamps = [snapshot['streams'][key]['stamp_ns'] for key in ('rgb', 'depth')]
    joint = snapshot['joint_stamp_ns']
    if type(now_ns) is not int or any(type(s) is not int or s <= 0 for s in [*stamps, joint]):
        raise ValueError('invalid_original_capture_stamps')
    if type(elapsed_s) not in (int, float) or not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError('invalid_processing_elapsed')
    ages = [(now_ns-s)/1e9 for s in stamps];joint_age = (now_ns-joint)/1e9
    images_ok = all(0 <= a <= .4 for a in ages) and abs(stamps[0]-stamps[1]) <= 30_000_000
    joints_ok = -.02 <= joint_age <= .4
    return dict(capture_to_result_age_s=max(ages),
        processing_including_save_s=elapsed_s, joint_age_at_result_s=joint_age,
        original_joint_stamp_ns=joint, freshness_checked_ns=now_ns,
        rgbd_freshness_passed=images_ok, joint_freshness_passed=joints_ok,
        freshness_passed=images_ok and joints_ok and elapsed_s <= .4,
        max_data_age_s=.4, original_snapshot_stamps_preserved=True,
        motion_authorized=False)


def enforce_final_timing(result, timing, observer=None):
    """Expire an in-flight result/seed if postprocessing used its remaining time."""
    result['timing'].update(timing)
    if timing['freshness_passed']:
        return
    result['eligible_fresh_candidate'] = False
    if 'center_evidence' in result:
        result['center_evidence'] = dict(coarse_center_consistent=False,
            reason='processed_capture_expired', motion_authorized=False, grasp_center_verified=False)
    if observer is not None:
        observer.track.reset()
        if observer.geometry is not None:
            observer.geometry.reset()
        result['identity_tracking'] = dict(status='reobserve', reason='processed_capture_expired',
            identity_confirmed=False, motion_authorized=False, grasp_center_verified=False)
